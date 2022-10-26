/*
 * This file is part of AtomGL.
 *
 * Copyright 2022 Davide Bettio <davide@uninstall.it>
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

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include <defaultatoms.h>
#include <interop.h>
#include <mailbox.h>
#include <term.h>

#include <esp32_sys.h>

#define DISPLAY_WIDTH 600
#define DISPLAY_HEIGHT 448

#define ENABLE_INIT_SPI_BUS CONFIG_AVM_DISPLAY_INIT_SPI_BUS

#define DISPLAY_BUSY 23
#define RESET_IO_NUM CONFIG_AVM_DISPLAY_RESET_IO_NUM
#define DISPLAY_DC CONFIG_AVM_DISPLAY_DC_IO_NUM

#include "display_items.h"
#include "font.c"
#include "spi_display.h"

#define CHAR_WIDTH 8

#define REPORT_UNEXPECTED_MSGS 0
#define CHECK_OVERFLOW 1
#define SELF_TEST 0

struct SPI
{
    struct SPIDisplay spi_disp;
    Context *ctx;
    xQueueHandle replies_queue;

    EventListener listener;
};

struct PendingReply
{
    uint64_t pending_call_ref_ticks;
    term pending_call_pid;
};

static xQueueHandle display_messages_queue;

static inline float square(float p)
{
    return p * p;
}

static uint8_t dither_acep7(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    const uint8_t m[4][4] = {
        { 0, 8, 2, 10 },
        { 12, 4, 14, 6 },
        { 3, 11, 1, 9 },
        { 15, 7, 13, 5 }
    };

    // following r parameters have been found using standard deviation
    // that gives a decent result
    int r1 = r + roundf(92.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    int g1 = g + roundf(85.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    int b1 = b + roundf(65.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));

    // values found by trial and error
    // they try to get closer to real colors than pure saturated RGB colors
    uint8_t colors[7][3] = {
        { 0x00, 0x00, 0x00 },
        { 0xFF, 0xFF, 0xFF },
        { 0x00, 0x80, 0x00 },
        { 0x00, 0x00, 0xFF },
        { 0xFF, 0x00, 0x00 },
        { 0xFf, 0xFF, 0x00 },
        { 0xFF, 0xAA, 0x00 }
    };

    float min = INT_MAX;
    int min_index = 0;

    for (int i = 0; i < 7; i++) {
        int r2 = colors[i][0];
        int g2 = colors[i][1];
        int b2 = colors[i][2];

#ifdef NO_WEIGHTS
        float d = square((r2 - r1)) + square((g2 - g1)) + square((b2 - b1));
#else
        float d = square((r2 - r1) * 0.30) + square((g2 - g1) * 0.59) + square((b2 - b1) * 0.11);
#endif

        if (d < min) {
            min = d;
            min_index = i;
        }
    }

    return min_index;
}

static void writecommand(struct SPIDisplay *spi_disp, uint8_t cmd)
{
    gpio_set_level(DISPLAY_DC, 0);
    spi_display_write(spi_disp, 8, cmd);
}

static void writedata(struct SPIDisplay *spi_disp, uint8_t data)
{
    gpio_set_level(DISPLAY_DC, 1);
    spi_display_write(spi_disp, 8, data);
}

static void display_reset()
{
    gpio_set_level(RESET_IO_NUM, 0);
    vTaskDelay(100);
    gpio_set_level(RESET_IO_NUM, 1);
}

static void wait_busy_level(int level)
{
    while (gpio_get_level(DISPLAY_BUSY) != level) {
        vTaskDelay(100);
    }
}

static inline void draw_pixel_x(uint8_t *line_buf, int xpos, uint8_t c)
{
#if CHECK_OVERFLOW
    if (xpos > DISPLAY_WIDTH) {
        fprintf(stderr, "buf ovf!\n");
        return;
    }
#endif

    if ((xpos & 1) == 0) {
        line_buf[xpos / 2] = (line_buf[xpos / 2] & 0xF) | (c << 4);
    } else {
        line_buf[xpos / 2] = (line_buf[xpos / 2] & 0xF0) | c;
    }
}

static int draw_image_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    int bgcolor_r;
    int bgcolor_g;
    int bgcolor_b;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor_r = (item->brcolor >> 24) & 0xFF;
        bgcolor_g = (item->brcolor >> 16) & 0xFF;
        bgcolor_b = (item->brcolor >> 8) & 0xFF;
        visible_bg = true;
    } else {
        bgcolor_r = 0;
        bgcolor_g = 0;
        bgcolor_b = 0;
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * width + (xpos - x);

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        if ((*pixels >> 24) & 0xFF) {
            uint8_t r = img_pixel >> 24;
            uint8_t g = (img_pixel >> 16) & 0xFF;
            uint8_t b = (img_pixel >> 8) & 0xFF;

            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, r, g, b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

static int draw_rect_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int width = item->width;

    uint8_t r = (item->brcolor >> 24) & 0xFF;
    uint8_t g = (item->brcolor >> 16) & 0xFF;
    uint8_t b = (item->brcolor >> 8) & 0xFF;

    int drawn_pixels = 0;

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, r, g, b);
        draw_pixel_x(line_buf, xpos + drawn_pixels, c);
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int draw_text_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;
    bool visible_bg;

    int fgcolor_r = (item->data.text_data.fgcolor >> 24) & 0xFF;
    int fgcolor_g = (item->data.text_data.fgcolor >> 16) & 0xFF;
    int fgcolor_b = (item->data.text_data.fgcolor >> 8) & 0xFF;

    int bgcolor_r;
    int bgcolor_g;
    int bgcolor_b;

    if (item->brcolor != 0) {
        bgcolor_r = (item->brcolor >> 24) & 0xFF;
        bgcolor_g = (item->brcolor >> 16) & 0xFF;
        bgcolor_b = (item->brcolor >> 8) & 0xFF;
        visible_bg = true;
    } else {
        bgcolor_r = 0;
        bgcolor_g = 0;
        bgcolor_b = 0;
        visible_bg = false;
    }

    char *text = (char *) item->data.text_data.text;

    int width = item->width;

    int drawn_pixels = 0;

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        int char_index = j / CHAR_WIDTH;
        char c = text[char_index];
        unsigned const char *glyph = fontdata + ((unsigned char) c) * 16;

        unsigned char row = glyph[ypos - y];

        bool opaque;
        int k = j % CHAR_WIDTH;
        if (row & (1 << (7 - k))) {
            opaque = true;
        } else {
            opaque = false;
        }

        if (opaque) {
            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, fgcolor_r, fgcolor_g, fgcolor_b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

static void do_update(Context *ctx, term display_list)
{
    int proper;
    int len = term_list_length(display_list, &proper);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);

    term t = display_list;
    for (int i = 0; i < len; i++) {
        init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = DISPLAY_WIDTH;
    int screen_height = DISPLAY_HEIGHT;
    struct SPI *spi = ctx->platform_data;

    struct SPIDisplay *spi_disp = &spi->spi_disp;
    spi_device_acquire_bus(spi_disp->handle, portMAX_DELAY);

    // resolution command
    writecommand(spi_disp, 0x61);
    writedata(spi_disp, 0x02);
    writedata(spi_disp, 0x58);
    writedata(spi_disp, 0x01);
    writedata(spi_disp, 0xC0);

    // update command
    writecommand(spi_disp, 0x10);

    gpio_set_level(DISPLAY_DC, 1);

    uint8_t *buf = heap_caps_malloc(DISPLAY_WIDTH / 2, MALLOC_CAP_DMA);
    memset(buf, 0, DISPLAY_WIDTH / 2);

    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
        }

        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(buf, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        spi_display_dmawrite(&spi->spi_disp, DISPLAY_WIDTH / 2, buf);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
    }

    // not sure if we should add 0x11, which is end of data command or not

    // power on command
    writecommand(spi_disp, 0x04);
    wait_busy_level(1);

    // refresh command
    writecommand(spi_disp, 0x12);
    wait_busy_level(1);

    // power off command
    writecommand(spi_disp, 0x02);
    spi_device_release_bus(spi_disp->handle);
    wait_busy_level(0);

    destroy_items(items, len);
}

static void process_message(Message *message, Context *ctx)
{
    term msg = message->message;

    term from = term_get_tuple_element(msg, 1);
    term req = term_get_tuple_element(msg, 2);
    term cmd = term_get_tuple_element(req, 0);

    struct SPI *spi = ctx->platform_data;

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {

        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);

    } else {
#if REPORT_UNEXPECTED_MSGS
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
#endif
    }

    term pid = term_get_tuple_element(from, 0);
    term ref = term_get_tuple_element(from, 1);

    struct PendingReply pending = {
        .pending_call_pid = pid,
        .pending_call_ref_ticks = term_to_ref_ticks(ref)
    };

    xQueueSend(spi->replies_queue, &pending, 1);
    xQueueSend(event_queue, &spi, 1);
}

static void process_messages(void *arg)
{
    struct SPI *args = arg;

    while (true) {
        Message *message;
        xQueueReceive(display_messages_queue, &message, portMAX_DELAY);
        process_message(message, args->ctx);
        free(message);
    }
}

static void send_message(term pid, term message, GlobalContext *global)
{
    int local_process_id = term_to_local_process_id(pid);
    Context *target = globalcontext_get_process(global, local_process_id);
    if (LIKELY(target)) {
        mailbox_send(target, message);
    }
}

static void display_callback(EventListener *listener)
{
    struct SPI *spi = listener->data;
    Context *ctx = spi->ctx;

    struct PendingReply pending;
    if (xQueueReceive(spi->replies_queue, &pending, 1)) {
        int reply_size = TUPLE_SIZE(2) + REF_SIZE + TUPLE_SIZE(3);
        if (UNLIKELY(memory_ensure_free(ctx, reply_size) != MEMORY_GC_OK)) {
            abort();
        }
        term from_tuple = term_alloc_tuple(2, ctx);
        term_put_tuple_element(from_tuple, 0, pending.pending_call_pid);
        term ref = term_from_ref_ticks(pending.pending_call_ref_ticks, ctx);
        term_put_tuple_element(from_tuple, 1, ref);

        term return_tuple = term_alloc_tuple(3, ctx);
        term_put_tuple_element(return_tuple, 0, context_make_atom(ctx, "\x6"
                                                                       "$reply"));
        term_put_tuple_element(return_tuple, 1, from_tuple);
        term_put_tuple_element(return_tuple, 2, OK_ATOM);

        send_message(pending.pending_call_pid, return_tuple, ctx->global);
    }
}

#if SELF_TEST
static void clear_screen(Context *ctx, int color)
{
    struct SPI *spi = ctx->platform_data;

    uint8_t *buf = heap_caps_malloc(DISPLAY_WIDTH / 2, MALLOC_CAP_DMA);

    struct SPIDisplay *spi_disp = &spi->spi_disp;
    spi_device_acquire_bus(spi_disp->handle, portMAX_DELAY);
    writecommand(spi_disp, 0x61);
    writedata(spi_disp, 0x02);
    writedata(spi_disp, 0x58);
    writedata(spi_disp, 0x01);
    writedata(spi_disp, 0xC0);
    writecommand(spi_disp, 0x10);

    gpio_set_level(DISPLAY_DC, 1);

    bool transaction_in_progress = false;

    for (int i = 0; i < DISPLAY_HEIGHT; i++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
        }

        memset(buf, color | (color << 4), DISPLAY_WIDTH / 2);
        spi_display_dmawrite(spi_disp, DISPLAY_WIDTH / 2, buf);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
    }

    writecommand(spi_disp, 0x04);
    wait_busy_level(1);
    writecommand(spi_disp, 0x12);
    wait_busy_level(1);
    writecommand(spi_disp, 0x02);
    spi_device_release_bus(spi_disp->handle);
    wait_busy_level(0);
}
#endif

static void display_spi_init(Context *ctx, term opts)
{
    struct SPI *spi = malloc(sizeof(struct SPI));
    // TODO check here

#if ENABLE_INIT_SPI_BUS == true
    spi_display_bus_init();
#endif

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.clock_speed_hz = 1000000;
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&spi->spi_disp, &spi_config);

    gpio_set_direction(19, GPIO_MODE_OUTPUT);
    gpio_set_level(19, 1);
    gpio_set_direction(DISPLAY_DC, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DISPLAY_DC, GPIO_PULLUP_ENABLE);
    gpio_set_direction(DISPLAY_BUSY, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DISPLAY_BUSY, GPIO_PULLUP_ENABLE);
    gpio_set_level(DISPLAY_DC, 0);

    esp_err_t ret = spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    display_reset();

    wait_busy_level(1);

    struct SPIDisplay *spi_disp = &spi->spi_disp;
    writecommand(spi_disp, 0x00);
    writedata(spi_disp, 0xEF);
    writedata(spi_disp, 0x08);
    writecommand(spi_disp, 0x01);
    writedata(spi_disp, 0x37);
    writedata(spi_disp, 0x00);
    writedata(spi_disp, 0x23);
    writedata(spi_disp, 0x23);
    writecommand(spi_disp, 0x03);
    writedata(spi_disp, 0x00);
    writecommand(spi_disp, 0x06);
    writedata(spi_disp, 0xC7);
    writedata(spi_disp, 0xC7);
    writedata(spi_disp, 0x1D);
    writecommand(spi_disp, 0x30);
    writedata(spi_disp, 0x3C);
    writecommand(spi_disp, 0x40);
    writedata(spi_disp, 0x00);
    writecommand(spi_disp, 0x50);
    writedata(spi_disp, 0x3F);
    writecommand(spi_disp, 0x60);
    writedata(spi_disp, 0x22);
    writecommand(spi_disp, 0x61);
    writedata(spi_disp, 0x02);
    writedata(spi_disp, 0x58);
    writedata(spi_disp, 0x01);
    writedata(spi_disp, 0xC0);
    writecommand(spi_disp, 0xE3);
    writedata(spi_disp, 0xAA);
    writecommand(spi_disp, 0x82);
    writedata(spi_disp, 0x80);

    vTaskDelay(10);

    writecommand(spi_disp, 0x50);
    writedata(spi_disp, 0x37);
    spi_device_release_bus(spi->spi_disp.handle);

    struct ESP32PlatformData *platform = ctx->global->platform_data;

    ctx->platform_data = spi;

    spi->ctx = ctx;
    spi->replies_queue = xQueueCreate(32, sizeof(struct PendingReply));
    spi->listener.sender = spi;
    spi->listener.data = spi;
    spi->listener.handler = display_callback;
    list_append(&platform->listeners, &spi->listener.listeners_list_head);

    display_messages_queue = xQueueCreate(32, sizeof(Message *));
    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);

#if SELF_TEST
    for (int i = 0; i < 7; i++) {
        fprintf(stderr, "color: %i\n", i);
        clear_screen(ctx, i);
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
    clear_screen(ctx, 1);

    while (1)
        ;
#endif
}

static void display_driver_consume_mailbox(Context *ctx)
{
    while (!list_is_empty(&ctx->mailbox)) {
        Message *message = mailbox_dequeue(ctx);
        xQueueSend(display_messages_queue, &message, 1);
    }
}

Context *acep_5in65_7c_display_driver_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_spi_init(ctx, opts);
    return ctx;
}
