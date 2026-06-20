/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 AtomGL contributors
 * Copyright 2026 Ibrahim YILMAZ <ibrahim@drlinux.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_driver.h"

#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <context.h>
#include <defaultatoms.h>
#include <interop.h>
#include <mailbox.h>
#include <port.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>

#include "dcs_lcd_draw.h"
#include "dcs_lcd_screen.h"
#include "display_items.h"
#include "display_message.h"
#include "display_task.h"

static const char *TAG = "rgb_lcd_display_driver";

struct RGBLCDDriver
{
    esp_lcd_panel_handle_t panel;
    struct DCSLCDScreen screen;
    uint16_t *framebuffers[3];
    size_t framebuffer_count;
    int active_fb_index;
    int previous_fb_index;
    uint16_t *cover_buffer;
    size_t cover_buffer_pixels;
    uint16_t *scaled_buffer;
    size_t scaled_buffer_pixels;
    uint16_t *background_buffer;
    size_t background_buffer_pixels;
    Context *ctx;
    struct DisplayTaskArgs display_args;
};

#define RGB_LCD_DRIVER_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct RGBLCDDriver, display_args)

static void surface_line_to_rgb565(uint16_t *line, int width)
{
    for (int i = 0; i < width; i++) {
        line[i] = __builtin_bswap16(line[i]);
    }
}

static uint16_t *active_framebuffer(struct RGBLCDDriver *driver)
{
    if (driver->active_fb_index < 0 || driver->active_fb_index >= (int) driver->framebuffer_count) {
        return NULL;
    }
    return driver->framebuffers[driver->active_fb_index];
}

static int select_work_framebuffer(struct RGBLCDDriver *driver)
{
    if (driver->framebuffer_count == 0) {
        return -1;
    }

    for (size_t i = 0; i < driver->framebuffer_count; i++) {
        if ((int) i != driver->active_fb_index && (int) i != driver->previous_fb_index) {
            return (int) i;
        }
    }
    for (size_t i = 0; i < driver->framebuffer_count; i++) {
        if ((int) i != driver->active_fb_index) {
            return (int) i;
        }
    }
    return driver->active_fb_index;
}

static esp_err_t switch_to_framebuffer(struct RGBLCDDriver *driver, int fb_index)
{
    if (fb_index < 0 || fb_index >= (int) driver->framebuffer_count) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(
        driver->panel, 0, 0, driver->screen.w, driver->screen.h, driver->framebuffers[fb_index]);
    if (err == ESP_OK) {
        driver->previous_fb_index = driver->active_fb_index;
        driver->active_fb_index = fb_index;
    }
    return err;
}

static void copy_rgb565_region_to_framebuffer(struct RGBLCDDriver *driver, int fb_index, int x, int y, int width, int height, const uint16_t *pixels)
{
    uint16_t *dst_fb = driver->framebuffers[fb_index];
    for (int row = 0; row < height; row++) {
        uint16_t *dst = dst_fb + ((y + row) * driver->screen.w) + x;
        memcpy(dst, pixels + ((size_t) row * width), (size_t) width * sizeof(uint16_t));
    }
}

static void mirror_line_to_inactive_framebuffers(struct RGBLCDDriver *driver, int y, int x0, int width, const uint16_t *line)
{
    if (driver->framebuffer_count <= 1) {
        return;
    }

    for (size_t i = 0; i < driver->framebuffer_count; i++) {
        if ((int) i == driver->active_fb_index) {
            continue;
        }
        uint16_t *fb = driver->framebuffers[i];
        if (!fb) {
            continue;
        }
        memcpy(fb + ((size_t) y * driver->screen.w) + x0, line, (size_t) width * sizeof(uint16_t));
    }
}

static void mirror_active_to_inactive_framebuffers(struct RGBLCDDriver *driver)
{
    if (driver->framebuffer_count <= 1) {
        return;
    }

    uint16_t *active = active_framebuffer(driver);
    if (!active) {
        return;
    }

    size_t fb_bytes = (size_t) driver->screen.w * (size_t) driver->screen.h * sizeof(uint16_t);
    for (size_t i = 0; i < driver->framebuffer_count; i++) {
        if ((int) i == driver->active_fb_index) {
            continue;
        }
        uint16_t *fb = driver->framebuffers[i];
        if (!fb) {
            continue;
        }
        memcpy(fb, active, fb_bytes);
    }
}

static void mirror_region_to_inactive_framebuffers(struct RGBLCDDriver *driver, int x, int y, int width, int height, const uint16_t *pixels)
{
    if (driver->framebuffer_count <= 1) {
        return;
    }

    for (size_t i = 0; i < driver->framebuffer_count; i++) {
        if ((int) i == driver->active_fb_index) {
            continue;
        }
        if (!driver->framebuffers[i]) {
            continue;
        }
        copy_rgb565_region_to_framebuffer(driver, (int) i, x, y, width, height, pixels);
    }
}
static void mirror_region_from_active_to_inactive_framebuffers(struct RGBLCDDriver *driver, int x, int y, int width, int height)
{
    if (driver->framebuffer_count <= 1) {
        return;
    }

    uint16_t *active = active_framebuffer(driver);
    if (!active) {
        return;
    }

    for (size_t i = 0; i < driver->framebuffer_count; i++) {
        if ((int) i == driver->active_fb_index) {
            continue;
        }
        uint16_t *fb = driver->framebuffers[i];
        if (!fb) {
            continue;
        }
        for (int row = 0; row < height; row++) {
            const uint16_t *src = active + ((size_t) (y + row) * driver->screen.w) + x;
            uint16_t *dst = fb + ((size_t) (y + row) * driver->screen.w) + x;
            memcpy(dst, src, (size_t) width * sizeof(uint16_t));
        }
    }
}

static bool ensure_cover_buffer(struct RGBLCDDriver *driver, int width, int height)
{
    size_t pixel_count = (size_t) width * (size_t) height;
    if (driver->cover_buffer && driver->cover_buffer_pixels >= pixel_count) {
        return true;
    }

    if (driver->cover_buffer) {
        heap_caps_free(driver->cover_buffer);
        driver->cover_buffer = NULL;
        driver->cover_buffer_pixels = 0;
    }

    driver->cover_buffer = heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!driver->cover_buffer) {
        driver->cover_buffer = heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_8BIT);
    }
    if (!driver->cover_buffer) {
        ESP_LOGE(TAG, "Failed to allocate cover buffer (%zu pixels).", pixel_count);
        return false;
    }

    driver->cover_buffer_pixels = pixel_count;
    return true;
}

static bool ensure_scaled_buffer(struct RGBLCDDriver *driver, int width, int height)
{
    size_t pixel_count = (size_t) width * (size_t) height;
    if (driver->scaled_buffer && driver->scaled_buffer_pixels >= pixel_count) {
        return true;
    }

    if (driver->scaled_buffer) {
        heap_caps_free(driver->scaled_buffer);
        driver->scaled_buffer = NULL;
        driver->scaled_buffer_pixels = 0;
    }

    driver->scaled_buffer = heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!driver->scaled_buffer) {
        driver->scaled_buffer = heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_8BIT);
    }
    if (!driver->scaled_buffer) {
        ESP_LOGE(TAG, "Failed to allocate scaled buffer (%zu pixels).", pixel_count);
        return false;
    }

    driver->scaled_buffer_pixels = pixel_count;
    return true;
}

static bool ensure_background_buffer(struct RGBLCDDriver *driver)
{
    size_t pixel_count = (size_t) driver->screen.w * (size_t) driver->screen.h;
    if (driver->background_buffer && driver->background_buffer_pixels >= pixel_count) {
        return true;
    }

    if (driver->background_buffer) {
        heap_caps_free(driver->background_buffer);
        driver->background_buffer = NULL;
        driver->background_buffer_pixels = 0;
    }

    driver->background_buffer = heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!driver->background_buffer) {
        driver->background_buffer = heap_caps_malloc(pixel_count * sizeof(uint16_t), MALLOC_CAP_8BIT);
    }
    if (!driver->background_buffer) {
        ESP_LOGE(TAG, "Failed to allocate background buffer (%zu pixels).", pixel_count);
        return false;
    }

    driver->background_buffer_pixels = pixel_count;
    return true;
}

static void maybe_store_fullscreen_background(
    struct RGBLCDDriver *driver, int x, int y, int width, int height, const uint16_t *pixels)
{
    if (x != 0 || y != 0 || width != driver->screen.w || height != driver->screen.h) {
        return;
    }
    if (!ensure_background_buffer(driver)) {
        return;
    }

    size_t bytes = (size_t) width * (size_t) height * sizeof(uint16_t);
    memcpy(driver->background_buffer, pixels, bytes);
    ESP_LOGI(TAG, "stored fullscreen background: %dx%d", width, height);
}

static bool restore_background_line_to_surface(struct RGBLCDDriver *driver, int y, int x0, int width)
{
    if (!driver->background_buffer) {
        return false;
    }

    memcpy(
        driver->screen.pixels + x0,
        driver->background_buffer + ((size_t) y * driver->screen.w) + x0,
        (size_t) width * sizeof(uint16_t));
    surface_line_to_rgb565(driver->screen.pixels + x0, width);
    return true;
}

static void scale_rgb565_nearest(
    const uint16_t *src, int src_w, int src_h, uint16_t *dst, int dst_w, int dst_h)
{
    for (int y = 0; y < dst_h; y++) {
        int src_y = ((y * src_h) + (dst_h / 2)) / dst_h;
        if (src_y >= src_h) {
            src_y = src_h - 1;
        }
        const uint16_t *src_row = src + ((size_t) src_y * src_w);
        uint16_t *dst_row = dst + ((size_t) y * dst_w);
        for (int x = 0; x < dst_w; x++) {
            int src_x = ((x * src_w) + (dst_w / 2)) / dst_w;
            if (src_x >= src_w) {
                src_x = src_w - 1;
            }
            dst_row[x] = src_row[src_x];
        }
    }
}

static bool display_list_to_items(Context *ctx, term display_list, BaseDisplayItem **out_items, int *out_len)
{
    int proper;
    int len = term_list_length(display_list, &proper);
    if (!proper || len < 0) {
        ESP_LOGE(TAG, "Invalid display list.");
        return false;
    }

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);
    if (UNLIKELY(!items)) {
        fprintf(stderr, "rgb display_list_to_items: failed to alloc items\n");
        return false;
    }

    term t = display_list;
    for (int i = 0; i < len; i++) {
        display_items_init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    *out_items = items;
    *out_len = len;
    return true;
}

static bool render_items_to_framebuffer(
    struct RGBLCDDriver *driver, int fb_index, int x0, int y0, int width, int height, BaseDisplayItem *items, int len)
{
    uint16_t *fb = driver->framebuffers[fb_index];
    for (int y = y0; y < y0 + height; y++) {
        (void) restore_background_line_to_surface(driver, y, x0, width);
        int x = x0;
        while (x < x0 + width) {
            int drawn_pixels = dcs_lcd_draw_x(&driver->screen, x, y, items, len);
            if (drawn_pixels <= 0) {
                ESP_LOGE(TAG, "Renderer stalled at x=%d y=%d.", x, y);
                return false;
            }
            if (x + drawn_pixels > x0 + width) {
                drawn_pixels = (x0 + width) - x;
            }
            x += drawn_pixels;
        }

        surface_line_to_rgb565(driver->screen.pixels + x0, width);
        memcpy(fb + ((size_t) y * driver->screen.w) + x0, driver->screen.pixels + x0, (size_t) width * sizeof(uint16_t));
    }
    return true;
}

static bool int_from_opts(term opts, const char *atom_str, int default_value, int *out, GlobalContext *global)
{
    term value = interop_kv_get_value_default(opts, atom_str, term_from_int(default_value), global);
    if (!term_is_integer(value)) {
        return false;
    }
    *out = term_to_int(value);
    return true;
}

static bool bool_from_opts(term opts, const char *atom_str, bool default_value, bool *out, GlobalContext *global)
{
    term value = interop_kv_get_value_default(
        opts, atom_str, default_value ? TRUE_ATOM : FALSE_ATOM, global);
    if (value == TRUE_ATOM) {
        *out = true;
        return true;
    }
    if (value == FALSE_ATOM) {
        *out = false;
        return true;
    }
    return false;
}

static bool parse_data_gpios(term opts, int data_gpios[16], GlobalContext *global)
{
    term key = globalcontext_make_atom(global, ATOM_STR("\xA", "data_gpios"));
    term value = interop_proplist_get_value(opts, key);
    if (value == term_nil()) {
        return false;
    }

    term list = value;
    for (int i = 0; i < 16; i++) {
        if (!term_is_nonempty_list(list)) {
            return false;
        }
        term head = term_get_list_head(list);
        if (!term_is_integer(head)) {
            return false;
        }
        data_gpios[i] = term_to_int(head);
        list = term_get_list_tail(list);
    }

    return list == term_nil();
}

static void do_update(Context *ctx, term display_list)
{
    struct RGBLCDDriver *driver = RGB_LCD_DRIVER_FROM_CTX(ctx);
    BaseDisplayItem *items = NULL;
    int len = 0;
    if (!display_list_to_items(ctx, display_list, &items, &len)) {
        return;
    }

    if (driver->framebuffer_count > 1) {
        int work_fb = select_work_framebuffer(driver);
        if (work_fb >= 0
                && render_items_to_framebuffer(driver, work_fb, 0, 0, driver->screen.w, driver->screen.h, items, len)) {
            esp_err_t err = switch_to_framebuffer(driver, work_fb);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "framebuffer switch failed: %s", esp_err_to_name(err));
            } else {
                // Keep non-active framebuffers aligned so cover-only swaps don't require full-frame copy.
                mirror_active_to_inactive_framebuffers(driver);
            }
        }
        display_items_delete(items, len);
        return;
    }

    for (int y = 0; y < driver->screen.h; y++) {
        (void) restore_background_line_to_surface(driver, y, 0, driver->screen.w);
        int x = 0;
        while (x < driver->screen.w) {
            int drawn_pixels = dcs_lcd_draw_x(&driver->screen, x, y, items, len);
            if (drawn_pixels <= 0) {
                ESP_LOGE(TAG, "Renderer stalled at x=%d y=%d.", x, y);
                display_items_delete(items, len);
                return;
            }
            x += drawn_pixels;
        }

        surface_line_to_rgb565(driver->screen.pixels, driver->screen.w);
        esp_err_t err = esp_lcd_panel_draw_bitmap(
            driver->panel, 0, y, driver->screen.w, y + 1, driver->screen.pixels);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
            break;
        }
    }

    display_items_delete(items, len);
}

static void do_update_region(Context *ctx, int x0, int y0, int width, int height, term display_list)
{
    struct RGBLCDDriver *driver = RGB_LCD_DRIVER_FROM_CTX(ctx);
    if (width <= 0 || height <= 0 || x0 >= driver->screen.w || y0 >= driver->screen.h) {
        return;
    }
    if (x0 < 0) {
        width += x0;
        x0 = 0;
    }
    if (y0 < 0) {
        height += y0;
        y0 = 0;
    }
    if (x0 + width > driver->screen.w) {
        width = driver->screen.w - x0;
    }
    if (y0 + height > driver->screen.h) {
        height = driver->screen.h - y0;
    }

    BaseDisplayItem *items = NULL;
    int len = 0;
    if (!display_list_to_items(ctx, display_list, &items, &len)) {
        return;
    }

    if (driver->framebuffer_count > 1) {
        int work_fb = select_work_framebuffer(driver);
        if (work_fb >= 0) {
            // Copy entire active framebuffer to the work buffer so content
            // outside the updated region (e.g. cover image drawn via
            // draw_buffer) is preserved across framebuffer switches.
            uint16_t *active_fb = active_framebuffer(driver);
            if (active_fb) {
                size_t fb_bytes = (size_t) driver->screen.w * (size_t) driver->screen.h * sizeof(uint16_t);
                memcpy(driver->framebuffers[work_fb], active_fb, fb_bytes);
            }
            if (render_items_to_framebuffer(driver, work_fb, x0, y0, width, height, items, len)) {
                esp_err_t err = switch_to_framebuffer(driver, work_fb);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "region framebuffer switch failed: %s", esp_err_to_name(err));
                } else {
                    mirror_region_from_active_to_inactive_framebuffers(driver, x0, y0, width, height);
                }
            }
        }
        display_items_delete(items, len);
        return;
    }

    for (int y = y0; y < y0 + height; y++) {
        (void) restore_background_line_to_surface(driver, y, x0, width);
        int x = x0;
        while (x < x0 + width) {
            int drawn_pixels = dcs_lcd_draw_x(&driver->screen, x, y, items, len);
            if (drawn_pixels <= 0) {
                ESP_LOGE(TAG, "Region renderer stalled at x=%d y=%d.", x, y);
                display_items_delete(items, len);
                return;
            }
            if (x + drawn_pixels > x0 + width) {
                drawn_pixels = (x0 + width) - x;
            }
            x += drawn_pixels;
        }

        surface_line_to_rgb565(driver->screen.pixels + x0, width);
        esp_err_t err = esp_lcd_panel_draw_bitmap(
            driver->panel, x0, y, x0 + width, y + 1, driver->screen.pixels + x0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "draw_bitmap region failed: %s", esp_err_to_name(err));
            break;
        }

        mirror_line_to_inactive_framebuffers(driver, y, x0, width, driver->screen.pixels + x0);
    }

    display_items_delete(items, len);
}

static int base64_value(uint8_t c)
{
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
        return c - '0' + 52;
    }
    if (c == '+') {
        return 62;
    }
    if (c == '/') {
        return 63;
    }
    if (c == '=') {
        return -2;
    }
    return -1;
}

static bool draw_rle_pixel(struct RGBLCDDriver *driver, int width, int height, uint32_t *out_index, uint16_t color)
{
    uint32_t pixel_count = (uint32_t) width * (uint32_t) height;
    if (*out_index >= pixel_count) {
        return true;
    }

    driver->cover_buffer[*out_index] = color;
    (*out_index)++;

    return true;
}

static bool feed_rle_byte(struct RGBLCDDriver *driver, int width, int height, uint32_t *out_index, uint8_t *rle_len, uint8_t rle_buf[3], uint8_t byte)
{
    rle_buf[(*rle_len)++] = byte;
    if (*rle_len < 3) {
        return true;
    }

    uint8_t count = rle_buf[0];
    uint16_t color = (uint16_t) rle_buf[1] | ((uint16_t) rle_buf[2] << 8);
    for (uint8_t i = 0; i < count; i++) {
        if (!draw_rle_pixel(driver, width, height, out_index, color)) {
            *rle_len = 0;
            return false;
        }
    }
    *rle_len = 0;
    return true;
}

static void do_draw_rgb565_rle_base64_scaled(
    Context *ctx, int x, int y, int src_width, int src_height, int dst_width, int dst_height, term b64_term)
{
    struct RGBLCDDriver *driver = RGB_LCD_DRIVER_FROM_CTX(ctx);
    if (!term_is_binary(b64_term)
            || src_width <= 0 || src_height <= 0
            || dst_width <= 0 || dst_height <= 0
            || x < 0 || y < 0
            || x + dst_width > driver->screen.w || y + dst_height > driver->screen.h) {
        ESP_LOGE(TAG, "Invalid draw_rgb565_rle_base64 arguments.");
        return;
    }
    if (!ensure_cover_buffer(driver, src_width, src_height)) {
        return;
    }

    const uint8_t *b64 = (const uint8_t *) term_binary_data(b64_term);
    size_t b64_len = term_binary_size(b64_term);
    uint8_t quad[4];
    uint8_t quad_len = 0;
    uint8_t rle_buf[3];
    uint8_t rle_len = 0;
    uint32_t out_index = 0;

    for (size_t i = 0; i < b64_len; i++) {
        int v = base64_value(b64[i]);
        if (v == -1) {
            continue;
        }
        quad[quad_len++] = b64[i];
        if (quad_len < 4) {
            continue;
        }

        int v0 = base64_value(quad[0]);
        int v1 = base64_value(quad[1]);
        int v2 = base64_value(quad[2]);
        int v3 = base64_value(quad[3]);
        if (v0 < 0 || v1 < 0 || v2 == -1 || v3 == -1) {
            ESP_LOGE(TAG, "Invalid base64 cover data.");
            return;
        }

        uint8_t out0 = (uint8_t) ((v0 << 2) | (v1 >> 4));
        if (!feed_rle_byte(driver, src_width, src_height, &out_index, &rle_len, rle_buf, out0)) {
            return;
        }
        if (v2 >= 0) {
            uint8_t out1 = (uint8_t) (((v1 & 0x0F) << 4) | (v2 >> 2));
            if (!feed_rle_byte(driver, src_width, src_height, &out_index, &rle_len, rle_buf, out1)) {
                return;
            }
        }
        if (v3 >= 0) {
            uint8_t out2 = (uint8_t) (((v2 & 0x03) << 6) | v3);
            if (!feed_rle_byte(driver, src_width, src_height, &out_index, &rle_len, rle_buf, out2)) {
                return;
            }
        }
        quad_len = 0;

        if (out_index >= (uint32_t) src_width * (uint32_t) src_height) {
            break;
        }
        if ((i & 0x1FFF) == 0x1FFF) {
            vTaskDelay(1);
        }
    }

    uint32_t pixel_count = (uint32_t) src_width * (uint32_t) src_height;
    if (out_index == pixel_count) {
        const uint16_t *draw_pixels = driver->cover_buffer;
        int draw_width = src_width;
        int draw_height = src_height;

        if (dst_width != src_width || dst_height != src_height) {
            if (!ensure_scaled_buffer(driver, dst_width, dst_height)) {
                return;
            }
            scale_rgb565_nearest(
                driver->cover_buffer, src_width, src_height, driver->scaled_buffer, dst_width, dst_height);
            draw_pixels = driver->scaled_buffer;
            draw_width = dst_width;
            draw_height = dst_height;
        }

        maybe_store_fullscreen_background(driver, x, y, draw_width, draw_height, draw_pixels);

        if (driver->framebuffer_count > 1) {
            int work_fb = select_work_framebuffer(driver);
            if (work_fb < 0) {
                ESP_LOGE(TAG, "cover framebuffer select failed.");
                return;
            }
            uint16_t *active_fb = active_framebuffer(driver);
            if (active_fb) {
                size_t fb_bytes = (size_t) driver->screen.w * (size_t) driver->screen.h * sizeof(uint16_t);
                memcpy(driver->framebuffers[work_fb], active_fb, fb_bytes);
            }
            copy_rgb565_region_to_framebuffer(driver, work_fb, x, y, draw_width, draw_height, draw_pixels);
            esp_err_t err = switch_to_framebuffer(driver, work_fb);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "cover framebuffer switch failed: %s", esp_err_to_name(err));
                return;
            }
            mirror_region_to_inactive_framebuffers(driver, x, y, draw_width, draw_height, draw_pixels);
        } else {
            esp_err_t err = esp_lcd_panel_draw_bitmap(
                driver->panel, x, y, x + draw_width, y + draw_height, draw_pixels);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "draw_bitmap cover failed: %s", esp_err_to_name(err));
                return;
            }
            mirror_region_to_inactive_framebuffers(driver, x, y, draw_width, draw_height, draw_pixels);
        }
    }

    ESP_LOGI(
        TAG, "cover RLE draw complete: %" PRIu32 "/%" PRIu32 " pixels (%dx%d -> %dx%d)",
        out_index, pixel_count, src_width, src_height, dst_width, dst_height);
}

static void do_draw_rgb565_rle_base64(Context *ctx, int x, int y, int width, int height, term b64_term)
{
    do_draw_rgb565_rle_base64_scaled(ctx, x, y, width, height, width, height, b64_term);
}

static void maybe_store_cover_background(
    struct RGBLCDDriver *driver, int x, int y, int width, int height, const uint16_t *pixels)
{
    if (!driver->background_buffer || x < 0 || y < 0 || width <= 0 || height <= 0) {
        return;
    }
    if (x + width > driver->screen.w || y + height > driver->screen.h) {
        return;
    }

    for (int row = 0; row < height; row++) {
        memcpy(
            driver->background_buffer + ((size_t) (y + row) * driver->screen.w) + x,
            pixels + ((size_t) row * width),
            (size_t) width * sizeof(uint16_t));
    }
}

static void draw_rgb565_region(struct RGBLCDDriver *driver, int x, int y, int width, int height, const uint16_t *pixels)
{
    if (driver->framebuffer_count > 1) {
        maybe_store_fullscreen_background(driver, x, y, width, height, pixels);
        int work_fb = select_work_framebuffer(driver);
        if (work_fb < 0) {
            ESP_LOGE(TAG, "draw_rgb565_region: framebuffer select failed.");
            return;
        }
        uint16_t *active_fb = active_framebuffer(driver);
        if (active_fb) {
            size_t fb_bytes = (size_t) driver->screen.w * (size_t) driver->screen.h * sizeof(uint16_t);
            memcpy(driver->framebuffers[work_fb], active_fb, fb_bytes);
        }
        copy_rgb565_region_to_framebuffer(driver, work_fb, x, y, width, height, pixels);
        esp_err_t err = switch_to_framebuffer(driver, work_fb);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "draw_rgb565_region: framebuffer switch failed: %s", esp_err_to_name(err));
            return;
        }
        mirror_region_to_inactive_framebuffers(driver, x, y, width, height, pixels);
    } else {
        maybe_store_fullscreen_background(driver, x, y, width, height, pixels);
        esp_err_t err = esp_lcd_panel_draw_bitmap(
            driver->panel, x, y, x + width, y + height, pixels);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "draw_rgb565_region: draw_bitmap failed: %s", esp_err_to_name(err));
            return;
        }
        mirror_region_to_inactive_framebuffers(driver, x, y, width, height, pixels);
    }

    maybe_store_cover_background(driver, x, y, width, height, pixels);
    ESP_LOGI(TAG, "draw_rgb565_region: %dx%d pixels at (%d,%d)", width, height, x, y);
}

static void process_message(Message *message, Context *ctx)
{
    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
        fprintf(stderr, "Received invalid message.");
        AVM_ABORT();
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        AVM_ABORT();
    }
    term cmd = term_get_tuple_element(req, 0);

    struct RGBLCDDriver *driver = RGB_LCD_DRIVER_FROM_CTX(ctx);

    if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x6", "update"))) {
        do_update(ctx, term_get_tuple_element(req, 1));
        return;

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\xD", "update_region"))) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        do_update_region(ctx, x, y, width, height, term_get_tuple_element(req, 5));
        return;

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x16", "draw_rgb565_rle_base64"))) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        do_draw_rgb565_rle_base64(ctx, x, y, width, height, term_get_tuple_element(req, 5));
        return;

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x1D", "draw_rgb565_rle_base64_scaled"))) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int src_width = term_to_int(term_get_tuple_element(req, 3));
        int src_height = term_to_int(term_get_tuple_element(req, 4));
        int dst_width = term_to_int(term_get_tuple_element(req, 5));
        int dst_height = term_to_int(term_get_tuple_element(req, 6));
        do_draw_rgb565_rle_base64_scaled(
            ctx, x, y, src_width, src_height, dst_width, dst_height, term_get_tuple_element(req, 7));
        return;

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\xB", "draw_buffer"))) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        term payload = term_get_tuple_element(req, 5);
        const uint16_t *data = NULL;

        if (term_is_binary(payload)) {
            size_t expected = (size_t) width * (size_t) height * 2;
            if (width <= 0 || height <= 0 || x < 0 || y < 0
                    || x + width > driver->screen.w || y + height > driver->screen.h
                    || term_binary_size(payload) < expected) {
                ESP_LOGE(TAG, "Invalid draw_buffer binary arguments.");
                return;
            }
            data = (const uint16_t *) term_binary_data(payload);
        } else {
            if (term_get_tuple_arity(req) < 7) {
                ESP_LOGE(TAG, "Invalid draw_buffer pointer arguments.");
                return;
            }
            unsigned long addr_low = term_to_int(payload);
            unsigned long addr_high = term_to_int(term_get_tuple_element(req, 6));
            data = (const uint16_t *) (addr_low | (addr_high << 16));
            if (!data || width <= 0 || height <= 0 || x < 0 || y < 0
                    || x + width > driver->screen.w || y + height > driver->screen.h) {
                ESP_LOGE(TAG, "Invalid draw_buffer pointer arguments.");
                return;
            }
        }

        draw_rgb565_region(driver, x, y, width, height, data);
        return;
    }

    fprintf(stderr, "rgb_display: ");
    term_display(stderr, req, ctx);
    fprintf(stderr, "\n");

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);
    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void rgb_lcd_free_driver(struct RGBLCDDriver *driver, bool delete_panel)
{
    if (!driver) {
        return;
    }
    if (delete_panel && driver->panel) {
        esp_lcd_panel_del(driver->panel);
    }
    if (driver->screen.pixels) {
        heap_caps_free(driver->screen.pixels);
    }
    free(driver);
}

static void display_init(Context *ctx, term opts)
{
    struct RGBLCDDriver *driver = calloc(1, sizeof(struct RGBLCDDriver));
    if (!driver) {
        ESP_LOGE(TAG, "Failed to allocate driver.");
        return;
    }
    driver->active_fb_index = 0;
    driver->previous_fb_index = -1;

    int width;
    int height;
    int pclk_hz;
    int hsync_pulse_width;
    int hsync_back_porch;
    int hsync_front_porch;
    int vsync_pulse_width;
    int vsync_back_porch;
    int vsync_front_porch;
    int hsync_gpio;
    int vsync_gpio;
    int de_gpio;
    int pclk_gpio;
    int bounce_buffer_size_px;
    bool pclk_active_neg;
    bool fb_in_psram;
    int data_gpios[16];

    bool ok = true;
    ok = ok && int_from_opts(opts, ATOM_STR("\x5", "width"), 800, &width, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x6", "height"), 480, &height, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x7", "pclk_hz"), 16000000, &pclk_hz, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x11", "hsync_pulse_width"), 4, &hsync_pulse_width, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x10", "hsync_back_porch"), 8, &hsync_back_porch, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x11", "hsync_front_porch"), 8, &hsync_front_porch, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x11", "vsync_pulse_width"), 4, &vsync_pulse_width, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x10", "vsync_back_porch"), 8, &vsync_back_porch, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x11", "vsync_front_porch"), 8, &vsync_front_porch, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\xA", "hsync_gpio"), 46, &hsync_gpio, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\xA", "vsync_gpio"), 3, &vsync_gpio, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x7", "de_gpio"), 5, &de_gpio, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x9", "pclk_gpio"), 7, &pclk_gpio, ctx->global);
    ok = ok && int_from_opts(opts, ATOM_STR("\x15", "bounce_buffer_size_px"), 8000, &bounce_buffer_size_px, ctx->global);
    ok = ok && bool_from_opts(opts, ATOM_STR("\xF", "pclk_active_neg"), true, &pclk_active_neg, ctx->global);
    ok = ok && bool_from_opts(opts, ATOM_STR("\xB", "fb_in_psram"), true, &fb_in_psram, ctx->global);
    ok = ok && parse_data_gpios(opts, data_gpios, ctx->global);

    if (!ok) {
        ESP_LOGE(TAG, "Failed init: invalid RGB LCD parameters.");
        free(driver);
        return;
    }

    driver->screen.w = width;
    driver->screen.h = height;
    driver->screen.pixels = heap_caps_malloc(width * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!driver->screen.pixels) {
        ESP_LOGE(TAG, "Failed to allocate scanline.");
        free(driver);
        return;
    }

    esp_lcd_rgb_panel_config_t config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .timings = {
            .pclk_hz = pclk_hz,
            .h_res = width,
            .v_res = height,
            .hsync_pulse_width = hsync_pulse_width,
            .hsync_back_porch = hsync_back_porch,
            .hsync_front_porch = hsync_front_porch,
            .vsync_pulse_width = vsync_pulse_width,
            .vsync_back_porch = vsync_back_porch,
            .vsync_front_porch = vsync_front_porch,
            .flags = {
                .pclk_active_neg = pclk_active_neg,
            },
        },
        .data_width = 16,
        .num_fbs = 2,
        .psram_trans_align = 64,
        .bounce_buffer_size_px = bounce_buffer_size_px,
        .hsync_gpio_num = hsync_gpio,
        .vsync_gpio_num = vsync_gpio,
        .de_gpio_num = de_gpio,
        .pclk_gpio_num = pclk_gpio,
        .disp_gpio_num = -1,
        .flags = {
            .fb_in_psram = fb_in_psram,
        },
    };

    for (int i = 0; i < 16; i++) {
        config.data_gpio_nums[i] = data_gpios[i];
    }

    esp_err_t err = esp_lcd_new_rgb_panel(&config, &driver->panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_rgb_panel failed: %s", esp_err_to_name(err));
        heap_caps_free(driver->screen.pixels);
        free(driver);
        return;
    }

    err = esp_lcd_panel_reset(driver->panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "panel reset failed: %s", esp_err_to_name(err));
        rgb_lcd_free_driver(driver, true);
        return;
    }
    err = esp_lcd_panel_init(driver->panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "panel init failed: %s", esp_err_to_name(err));
        rgb_lcd_free_driver(driver, true);
        return;
    }

    void *fb0 = NULL;
    void *fb1 = NULL;
    err = esp_lcd_rgb_panel_get_frame_buffer(driver->panel, 2, &fb0, &fb1);
    if (err == ESP_OK && fb0 && fb1) {
        driver->framebuffers[0] = fb0;
        driver->framebuffers[1] = fb1;
        driver->framebuffer_count = 2;
        ESP_LOGI(TAG, "Using RGB LCD double framebuffer: %p %p", fb0, fb1);
    } else {
        ESP_LOGW(TAG, "RGB LCD multi-framebuffer unavailable, using draw_bitmap path: %s", esp_err_to_name(err));
    }

    driver->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    driver->display_args.process_message_fn = process_message;
    driver->display_args.ctx = ctx;
    driver->ctx = ctx;
    ctx->platform_data = &driver->display_args;

    xTaskCreate(display_task_process_messages, "display", 10000, &driver->display_args, 1, NULL);
}

Context *rgb_lcd_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
}
