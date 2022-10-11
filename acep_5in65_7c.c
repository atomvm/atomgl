/*
 * This file is part of AtomVM.
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
#include <string.h>

#include <defaultatoms.h>
#include <interop.h>
#include <mailbox.h>
#include <term.h>

#include <esp32_sys.h>

#define DISPLAY_WIDTH 600
#define DISPLAY_HEIGHT 448

#define DISPLAY_CS 5
#define DISPLAY_BUSY 23
#define DISPLAY_DC 27

#include "display_items.h"
#include "font.c"

#define CHAR_WIDTH 8

#define REPORT_UNEXPECTED_MSGS 0
#define CHECK_OVERFLOW 1
#define SELF_TEST 0

struct SPI
{
    spi_device_handle_t handle;
    spi_transaction_t transaction;
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

static spi_device_handle_t handle;

static inline float square(float p)
{
    return p * p;
}

static inline void dither(int x, int y, uint8_t r, uint8_t g, uint8_t b, int *out_r, int *out_g, int *out_b)
{
    uint8_t m[4][4] = {
        { 0, 8, 2, 10 },
        { 12, 4, 14, 6 },
        { 3, 11, 1, 9 },
        { 15, 7, 13, 5 }
    };

    // following r parameters have been found using standard deviation
    // that gives a decent result
    *out_r = r + roundf(92.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    *out_g = g + roundf(85.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    *out_b = b + roundf(65.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
}

static int closest(int r1, int g1, int b1)
{
    // values found by trial and error
    // they try to get closer to real colors than pure saturated RGB colors
    uint8_t colors[7][3] = {
        { 0x0D, 0x0B, 0x0E },
        { 0xD7, 0xD4, 0xD1 },
        { 0x2C, 0x66, 0x2D },
        { 0x31, 0x32, 0x7F },
        { 0x8F, 0x2A, 0x30 },
        { 0xED, 0xE8, 0x58 },
        { 0xC6, 0x8C, 0x45 }
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

static bool spiwrite(int data_len, uint32_t data)
{
    spi_transaction_t transaction;

    memset(&transaction, 0, sizeof(spi_transaction_t));

    uint32_t tx_data = SPI_SWAP_DATA_TX(data, data_len);

    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = data_len;
    transaction.addr = 0;
    transaction.tx_data[0] = tx_data;
    transaction.tx_data[1] = (tx_data >> 8) & 0xFF;
    transaction.tx_data[2] = (tx_data >> 16) & 0xFF;
    transaction.tx_data[3] = (tx_data >> 24) & 0xFF;

    // TODO: int ret = spi_device_queue_trans(spi_data->handle, &spi_data->transaction, portMAX_DELAY);
    int ret = spi_device_polling_transmit(handle, &transaction);
    if (UNLIKELY(ret != ESP_OK)) {
        fprintf(stderr, "spiwrite: transmit error\n");
        return false;
    }

    return true;
}

static bool spidmawrite(int data_len, const void *data)
{
    spi_transaction_t transaction;

    memset(&transaction, 0, sizeof(spi_transaction_t));

    transaction.flags = 0;
    transaction.length = data_len * 8;
    transaction.addr = 0;
    transaction.tx_buffer = data;

    // TODO: int ret = spi_device_queue_trans(spi_data->handle, &spi_data->transaction, portMAX_DELAY);
    int ret = spi_device_polling_transmit(handle, &transaction);
    if (UNLIKELY(ret != ESP_OK)) {
        fprintf(stderr, "spidmawrite: transmit error\n");
        return false;
    }

    return true;
}

static void writecommand(uint8_t cmd)
{
    gpio_set_level(DISPLAY_DC, 0);
    spiwrite(8, cmd);
}

static void writedata(uint8_t data)
{
    gpio_set_level(DISPLAY_DC, 1);
    spiwrite(8, data);
}

static void display_reset()
{
    gpio_set_level(19, 0);
    vTaskDelay(100);
    gpio_set_level(19, 1);
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
            int rd;
            int gd;
            int bd;
            dither(xpos, ypos, r, g, b, &rd, &gd, &bd);
            uint8_t c = closest(rd, gd, bd);

            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            int rd;
            int gd;
            int bd;
            dither(xpos, ypos, bgcolor_r, bgcolor_g, bgcolor_b, &rd, &gd, &bd);
            uint8_t c = closest(rd, gd, bd);

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
        int rd;
        int gd;
        int bd;
        dither(xpos + j, ypos, r, g, b, &rd, &gd, &bd);
        uint8_t c = closest(rd, gd, bd);

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
            int rd;
            int gd;
            int bd;
            dither(xpos + drawn_pixels, ypos, fgcolor_r, fgcolor_g, fgcolor_b, &rd, &gd, &bd);
            uint8_t c = closest(rd, gd, bd);

            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            int rd;
            int gd;
            int bd;
            dither(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b, &rd, &gd, &bd);
            uint8_t c = closest(rd, gd, bd);

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

    spi_device_acquire_bus(handle, portMAX_DELAY);
    writecommand(0x61);
    writedata(0x02);
    writedata(0x58);
    writedata(0x01);
    writedata(0xC0);
    writecommand(0x10);

    gpio_set_level(DISPLAY_DC, 1);

    uint8_t *buf = heap_caps_malloc(DISPLAY_WIDTH / 2, MALLOC_CAP_DMA);
    memset(buf, 0, DISPLAY_WIDTH / 2);

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(buf, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        spidmawrite(DISPLAY_WIDTH / 2, buf);
    }

    writecommand(0x04);
    wait_busy_level(1);
    writecommand(0x12);
    wait_busy_level(1);
    writecommand(0x02);
    spi_device_release_bus(handle);
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
static void clear_screen(int color)
{
    uint8_t *buf = heap_caps_malloc(DISPLAY_WIDTH / 2, MALLOC_CAP_DMA);

    spi_device_acquire_bus(handle, portMAX_DELAY);
    writecommand(0x61);
    writedata(0x02);
    writedata(0x58);
    writedata(0x01);
    writedata(0xC0);
    writecommand(0x10);

    gpio_set_level(DISPLAY_DC, 1);

    for (int i = 0; i < DISPLAY_HEIGHT; i++) {
        memset(buf, color | (color << 4), DISPLAY_WIDTH / 2);
        spidmawrite(DISPLAY_WIDTH / 2, buf);
    }
    writecommand(0x04);
    wait_busy_level(1);
    writecommand(0x12);
    wait_busy_level(1);
    writecommand(0x02);
    spi_device_release_bus(handle);
    wait_busy_level(0);
}
#endif

static void display_spi_init(Context *ctx)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = 25,
        .sclk_io_num = 26,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = 5,
        .queue_size = 1
    };
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
    ESP_ERROR_CHECK(ret);

    gpio_set_direction(19, GPIO_MODE_OUTPUT);
    gpio_set_level(19, 1);
    gpio_set_direction(DISPLAY_DC, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DISPLAY_DC, GPIO_PULLUP_ENABLE);
    gpio_set_direction(DISPLAY_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(DISPLAY_BUSY, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DISPLAY_BUSY, GPIO_PULLUP_ENABLE);
    gpio_set_level(DISPLAY_DC, 0);
    gpio_set_level(DISPLAY_CS, 0);

    ret = spi_device_acquire_bus(handle, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    display_reset();

    wait_busy_level(1);

    writecommand(0x00);
    writedata(0xEF);
    writedata(0x08);
    writecommand(0x01);
    writedata(0x37);
    writedata(0x00);
    writedata(0x23);
    writedata(0x23);
    writecommand(0x03);
    writedata(0x00);
    writecommand(0x06);
    writedata(0xC7);
    writedata(0xC7);
    writedata(0x1D);
    writecommand(0x30);
    writedata(0x3C);
    writecommand(0x40);
    writedata(0x00);
    writecommand(0x50);
    writedata(0x3F);
    writecommand(0x60);
    writedata(0x22);
    writecommand(0x61);
    writedata(0x02);
    writedata(0x58);
    writedata(0x01);
    writedata(0xC0);
    writecommand(0xE3);
    writedata(0xAA);
    writecommand(0x82);
    writedata(0x80);

    vTaskDelay(10);

    writecommand(0x50);
    writedata(0x37);
    spi_device_release_bus(handle);

    struct ESP32PlatformData *platform = ctx->global->platform_data;

    struct SPI *spi = malloc(sizeof(struct SPI));
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
        clear_screen(i);
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
    clear_screen(1);

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
    display_spi_init(ctx);
    return ctx;
}
