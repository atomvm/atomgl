/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2024 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_driver.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#include <atom.h>
#include <bif.h>
#include <context.h>
#include <debug.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <module.h>
#include <port.h>
#include <sys.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>

#include <trace.h>

#include "backlight_gpio.h"
#include "dcs_lcd_color.h"
#include "dcs_lcd_commands.h"
#include "dcs_lcd_draw.h"
#include "dcs_lcd_screen.h"
#include "display_common.h"
#include "display_items.h"
#include "display_message.h"
#include "display_task.h"
#include "image_helpers.h"
#include "spi_dc_driver.h"
#include "spi_display.h"

#define SPI_MODE 0

#include "font_data.h"

static const char *TAG = "dcs_lcd_display_driver";

static inline void delay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

struct DCSLCDDriver
{
    struct SPIDCBus bus;
    int reset_gpio;

    avm_int_t rotation;
    bool madctl_bgr;

    const struct DCSLCDDesc *desc;

    struct DCSLCDScreen screen;

    Context *ctx;

    struct DisplayTaskArgs display_args;
};

static const struct {
    const char *compat;
    const struct DCSLCDDesc *desc;
} dcs_lcd_compat_table[] = {
    { "ilitek,ili9341",   &dcs_lcd_desc_ili9341 },
    { "ilitek,ili9342c",  &dcs_lcd_desc_ili9342c },
    { "ilitek,ili9486",   &dcs_lcd_desc_ili9486 },
    { "ilitek,ili9488",   &dcs_lcd_desc_ili9488 },
    { "sitronix,st7789",  &dcs_lcd_desc_st7789 },
    { "sitronix,st7796",  &dcs_lcd_desc_st7796 },
};

static const struct DCSLCDDesc *dcs_lcd_desc_for_compatible(const char *compat)
{
    for (size_t i = 0; i < sizeof(dcs_lcd_compat_table) / sizeof(dcs_lcd_compat_table[0]); i++) {
        if (!strcmp(compat, dcs_lcd_compat_table[i].compat)) {
            return dcs_lcd_compat_table[i].desc;
        }
    }
    return NULL;
}

// ILI9488 scanline conversion: RGB565 -> RGB888 bytes.
static inline void rgb565_swapped_line_to_rgb888(uint8_t *dst, const uint16_t *src_swapped, int n_pixels)
{
    for (int i = 0; i < n_pixels; i++) {
        uint16_t px = (uint16_t) SPI_SWAP_DATA_TX(src_swapped[i], 16);

        uint8_t r5 = (px >> 11) & 0x1F;
        uint8_t g6 = (px >> 5) & 0x3F;
        uint8_t b5 = (px >> 0) & 0x1F;

        uint8_t r8 = (r5 << 3) | (r5 >> 2);
        uint8_t g8 = (g6 << 2) | (g6 >> 4);
        uint8_t b8 = (b5 << 3) | (b5 >> 2);

        dst[i * 3 + 0] = r8;
        dst[i * 3 + 1] = g8;
        dst[i * 3 + 2] = b8;
    }
}

#define DCS_LCD_DRIVER_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct DCSLCDDriver, display_args)

static void display_init(Context *ctx, term opts);
static void display_init_using_list(struct DCSLCDDriver *driver, term init_list);

static void do_update(Context *ctx, term display_list)
{
    BaseDisplayItem *items;
    size_t len;
    if (UNLIKELY(display_items_new_list(display_list, &items, &len, ctx) != DisplayItemsOk)) {
        return;
    }

    struct DCSLCDDriver *driver = DCS_LCD_DRIVER_FROM_CTX(ctx);
    int screen_width = driver->screen.w;
    int screen_height = driver->screen.h;

    dcs_lcd_set_paint_area(&driver->bus, &driver->screen, 0, 0, screen_width, screen_height);
    spi_dc_write_command(&driver->bus, DCS_LCD_RAMWR);
    spi_device_acquire_bus(driver->bus.spi_disp.handle, portMAX_DELAY);

    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = dcs_lcd_draw_x(&driver->screen, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        if (transaction_in_progress) {
            spi_transaction_t *trans;
            // I did a quick measurement, and most of the time is spent waiting for DMA transaction
            // eg. 23 us spent in draw_x, 188 us spent in spi_device_get_trans_result
            spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        // Swap scanline buffers.
        void *tmp = driver->screen.pixels;
        driver->screen.pixels = driver->screen.pixels_out;
        driver->screen.pixels_out = tmp;

        if (driver->desc->pixel_bytes == 2) {
            spi_display_dma_write(&driver->bus.spi_disp, screen_width * sizeof(uint16_t), driver->screen.pixels_out);
        } else {
            void *tmpb = driver->screen.bytes;
            driver->screen.bytes = driver->screen.bytes_out;
            driver->screen.bytes_out = tmpb;

            rgb565_swapped_line_to_rgb888(driver->screen.bytes_out, driver->screen.pixels_out, screen_width);
            spi_display_dma_write(&driver->bus.spi_disp, screen_width * 3, driver->screen.bytes_out);
        }

        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans;
        spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(driver->bus.spi_disp.handle);

    display_items_delete(items, len);
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

    struct DCSLCDDriver *driver = DCS_LCD_DRIVER_FROM_CTX(ctx);

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {
        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);
        return;

    } else if (cmd == context_make_atom(ctx, "\xB"
                                             "draw_buffer")) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        unsigned long addr_low = term_to_int(term_get_tuple_element(req, 5));
        unsigned long addr_high = term_to_int(term_get_tuple_element(req, 6));

        const void *data = (const void *) ((addr_low | (addr_high << 16)));

        dcs_lcd_draw_buffer(&driver->bus, &driver->screen, driver->desc->pixel_bytes, x, y, width, height, data);

        // Reply already sent at enqueue time by
        // try_pre_ack_render_cmd in display_task.c.
        return;

    } else if (cmd == globalcontext_make_atom(ctx->global, "\xA" "load_image")) {
        handle_load_image(req, gen_message.ref, gen_message.pid, ctx);
        return;

    } else {
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
    }

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);

    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void set_rotation(struct DCSLCDDriver *driver, int rotation)
{
    uint8_t madctl = driver->desc->madctl[rotation & 3];
    if (driver->madctl_bgr) {
        madctl |= DCS_LCD_MAD_BGR;
    }
    spi_dc_write_command(&driver->bus, DCS_LCD_MADCTL);
    spi_dc_write_data(&driver->bus, madctl);
}

Context *dcs_lcd_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
}

static void display_init(Context *ctx, term opts)
{
    // Resolve compatible string -> per-controller descriptor.
    term compat_term = interop_kv_get_value_default(
        opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_term, &str_ok);
    const struct DCSLCDDesc *desc = NULL;
    if (str_ok && compat_string) {
        desc = dcs_lcd_desc_for_compatible(compat_string);
    }
    if (!desc) {
        ESP_LOGE(TAG, "Failed init: unknown or missing compatible '%s'.",
            compat_string ? compat_string : "(null)");
        free(compat_string);
        return;
    }
    free(compat_string);

    term rotation_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);
    bool ok = term_is_integer(rotation_term);
    avm_int_t rotation = ok ? term_to_int(rotation_term) : 0;

    // Default geometry from descriptor, swapped on odd rotation, overrideable.
    int default_w = (rotation & 1) ? desc->native_height : desc->native_width;
    int default_h = (rotation & 1) ? desc->native_width : desc->native_height;
    term width_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x5", "width"), term_from_int(default_w), ctx->global);
    term height_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x6", "height"), term_from_int(default_h), ctx->global);

    struct DCSLCDDriver *driver = calloc(1, sizeof(struct DCSLCDDriver));
    driver->desc = desc;
    driver->rotation = rotation;
    driver->madctl_bgr = desc->default_bgr;
    driver->screen.w = term_to_int(width_term);
    driver->screen.h = term_to_int(height_term);
    driver->screen.pixels = heap_caps_malloc(driver->screen.w * sizeof(uint16_t), MALLOC_CAP_DMA);
    driver->screen.pixels_out = heap_caps_malloc(driver->screen.w * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (desc->pixel_bytes == 3) {
        driver->screen.bytes = heap_caps_malloc(driver->screen.w * 3, MALLOC_CAP_DMA);
        driver->screen.bytes_out = heap_caps_malloc(driver->screen.w * 3, MALLOC_CAP_DMA);
    }

    driver->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    driver->display_args.process_message_fn = process_message;
    driver->display_args.ctx = ctx;
    ctx->platform_data = &driver->display_args;

    driver->ctx = ctx;

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.mode = SPI_MODE;
    spi_config.clock_speed_hz = desc->spi_clock_hz;
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&driver->bus.spi_disp, &spi_config);

    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &driver->bus.dc_gpio, ctx->global);

    bool reset_configured = true;
    if (!display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &driver->reset_gpio, ctx->global)) {
        ESP_LOGI(TAG, "Reset GPIO not configured.");
        reset_configured = false;
    }

    term invon = interop_kv_get_value_default(opts, ATOM_STR("\x10", "enable_tft_invon"), FALSE_ATOM, ctx->global);
    ok = ok && ((invon == TRUE_ATOM) || (invon == FALSE_ATOM));
    bool enable_tft_invon = (invon == TRUE_ATOM);

    // color_order: rgb|bgr (default: per-descriptor)
    term color_order_term = interop_kv_get_value_default(opts, ATOM_STR("\xB", "color_order"), term_nil(), ctx->global);
    if (color_order_term != term_nil()) {
        if (color_order_term == context_make_atom(ctx, "\x3" "rgb")) {
            driver->madctl_bgr = false;
        } else if (color_order_term == context_make_atom(ctx, "\x3" "bgr")) {
            driver->madctl_bgr = true;
        } else {
            ok = false;
        }
    }

    term x_off_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x8", "x_offset"), term_from_int(0), ctx->global);
    term y_off_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x8", "y_offset"), term_from_int(0), ctx->global);

    if (term_is_integer(x_off_term) && term_is_integer(y_off_term)) {
        driver->screen.x_offset = (int16_t) term_to_int(x_off_term);
        driver->screen.y_offset = (int16_t) term_to_int(y_off_term);
    } else {
        ok = false;
    }

    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display parameters.");
        return;
    }

    if (desc->madctl[driver->rotation & 3] == 0xFF) {
        ESP_LOGE(TAG, "Failed init: rotation %d not supported by controller %s.",
            (int) driver->rotation, desc->name);
        return;
    }

    // Reset
    if (reset_configured) {
        spi_device_acquire_bus(driver->bus.spi_disp.handle, portMAX_DELAY);
        gpio_set_direction(driver->reset_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(driver->reset_gpio, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(driver->reset_gpio, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(driver->reset_gpio, 1);
        spi_device_release_bus(driver->bus.spi_disp.handle);
    }

    gpio_set_direction(driver->bus.dc_gpio, GPIO_MODE_OUTPUT);

    // Init sequence: init_list opt overrides everything; otherwise descriptor
    // default, with init_seq_type "alt_gamma_2" selecting the ST7789 alt seq.
    term maybe_init_list
        = interop_kv_get_value_default(opts, ATOM_STR("\x9", "init_list"), term_nil(), ctx->global);
    if (maybe_init_list != term_nil()) {
        display_init_using_list(driver, maybe_init_list);
    } else {
        const uint8_t *init_seq = desc->default_init_seq;
        term init_seq_type_term = interop_kv_get_value_default(
            opts, ATOM_STR("\xD", "init_seq_type"), term_nil(), ctx->global);
        if (init_seq_type_term != term_nil()) {
            int type_ok;
            char *type_str = interop_term_to_string(init_seq_type_term, &type_ok);
            if (type_ok && !strcmp(type_str, "alt_gamma_2")
                && (desc == &dcs_lcd_desc_st7789 || desc == &dcs_lcd_desc_st7796)) {
                init_seq = dcs_lcd_init_seq_st7789_alt;
            }
            free(type_str);
        }
        dcs_lcd_execute_init_seq(&driver->bus, init_seq);
    }

    set_rotation(driver, driver->rotation);

    spi_dc_write_command(&driver->bus, enable_tft_invon ? DCS_LCD_INVON : DCS_LCD_INVOFF);

    struct BacklightGPIOConfig backlight_config;
    backlight_gpio_init_config(&backlight_config);
    backlight_gpio_parse_config(&backlight_config, opts, ctx->global);
    backlight_gpio_init(&backlight_config);

    xTaskCreate(display_task_process_messages, "display", 10000, &driver->display_args, 1, NULL);
}

static void display_init_using_list(struct DCSLCDDriver *driver, term init_list)
{
    term t = init_list;
    while (term_is_nonempty_list(t)) {
        term head = term_get_list_head(t);
        if (term_is_tuple(head) && term_get_tuple_arity(head) == 2) {
            term cmd_term = term_get_tuple_element(head, 0);
            term data_term = term_get_tuple_element(head, 1);
            if (term_is_integer(cmd_term) && term_is_binary(data_term)) {
                avm_int_t cmd = term_to_int(cmd_term);
                const uint8_t *data = (const uint8_t *) term_binary_data(data_term);
                spi_dc_write_cmd_data(&driver->bus, cmd, data, term_binary_size(data_term));
            } else if ((cmd_term == context_make_atom(driver->ctx, ATOM_STR("\x8", "sleep_ms")))
                && term_is_integer(data_term)) {
                delay(term_to_int(data_term));
            } else {
                // invalid
                break;
            }
        } else {
            // invalid
            break;
        }

        t = term_get_list_tail(t);
    }
    if (t != term_nil()) {
        fprintf(stderr, "Invalid init_list!\n");
    }
}
