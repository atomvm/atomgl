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

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

#include <driver/gpio.h>

#include <atom.h>
#include <bif.h>
#include <context.h>
#include <debug.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <module.h>
#include <sys.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>

#include <trace.h>

#include <math.h>

#include "display_common.h"
#include "spi_display.h"

#define CHAR_WIDTH 8

#define DISPLAY_WIDTH 400

#define CHECK_OVERFLOW 1
#define REPORT_UNEXPECTED_MSGS 0

#define ENABLE_INIT_SPI_BUS CONFIG_AVM_DISPLAY_INIT_SPI_BUS

#include "font.c"

struct SPI
{
    struct SPIDisplay spi_disp;
    Context *ctx;
    xQueueHandle replies_queue;

    EventListener listener;
};

#include "display_items.h"

// This struct is just for compatibility reasons with the SDL display driver
// so it is possible to easily copy & paste code from there.
struct Screen
{
    int w;
    int h;
    uint8_t *pixels;
    uint8_t *dma_out;
    // keep double buffer disabled for now: uint16_t *pixels_out;
};

struct Screen *screen;

struct PendingReply
{
    uint64_t pending_call_ref_ticks;
    term pending_call_pid;
};

static xQueueHandle display_messages_queue;

static void display_driver_consume_mailbox(Context *ctx);
static void display_init(Context *ctx, term opts);

int vcom = 0x0;
static inline int get_vcom()
{
    int current_vcom = vcom;
    if (!vcom) {
        vcom = 0x2;
    } else {
        vcom = 0;
    }

    return current_vcom;
}

static int get_color(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    // dither

    /*
     * Original bayer matrix
     *   { 0, 8, 2, 10 },
     *   { 12, 4, 14, 6 },
     *   { 3, 11, 1, 9 },
     *   { 15, 7, 13, 5 }
     *
     *   The following is calculated applying the following code element by element
     *   r = 255 / values / 4
     *   roundf(63.75 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
     */
    const int m[4][4] = {
        { -32, 0, -24, 8 },
        { 16, -16, 24, -8 },
        { -20, 12, -28, 4 },
        { 28, -4, 20, -12 }
    };

    int v = m[x % 4][y % 4];
    int out_r = r + v;
    int out_g = g + v;
    int out_b = b + v;
    // end of dither

    // get closest
    // float yval = 0.2126 * out_r + 0.7152 * out_g + 0.0722 * out_b;
    // the following is a fast formula
    int yval = ((out_r << 1) + out_r + (out_g << 2) + out_b) >> 3;

    return yval >= 128;
}

static inline void draw_pixel_x(uint8_t *line_buf, int xpos, int color)
{
#if CHECK_OVERFLOW
    if (xpos > DISPLAY_WIDTH) {
        fprintf(stderr, "display buffer overflow: %i!\n", xpos);
        return;
    }
#endif

    int bpos = (xpos % 8);
    line_buf[xpos / 8] = (line_buf[xpos / 8] & ~(0x1 << bpos)) | (color << bpos);
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

            uint8_t c = get_color(xpos + drawn_pixels, ypos, r, g, b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = get_color(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
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
        uint8_t c = get_color(xpos + drawn_pixels, ypos, r, g, b);
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
            uint8_t c = get_color(xpos + drawn_pixels, ypos, fgcolor_r, fgcolor_g, fgcolor_b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = get_color(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
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

    int screen_width = screen->w;
    int screen_height = screen->h;
    struct SPI *spi = ctx->platform_data;

    int memsize = 2 + 400 / 8 + 2;
    uint8_t *buf = screen->pixels;

    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (!screen->dma_out && transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
        }

        memset(buf + 2, 0xFF, DISPLAY_WIDTH / 8);

        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(buf + 2, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        buf[0] = 0x1 | get_vcom();
        buf[1] = ypos + 1;
        buf[2 + DISPLAY_WIDTH / 8] = 0;
        buf[2 + DISPLAY_WIDTH / 8 + 1] = 0;

        if (screen->dma_out) {
            if (transaction_in_progress) {
                spi_transaction_t *trans = NULL;
                spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
            }
            void *tmp = screen->pixels;
            screen->pixels = screen->dma_out;
            buf = screen->pixels;
            screen->dma_out = tmp;

            spi_display_dmawrite(&spi->spi_disp, memsize, screen->dma_out);
        } else {
            spi_display_dmawrite(&spi->spi_disp, memsize, buf);
        }

        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans;
        spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(spi->spi_disp.handle);
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

static void display_driver_consume_mailbox(Context *ctx)
{
    while (!list_is_empty(&ctx->mailbox)) {
        Message *message = mailbox_dequeue(ctx);
        xQueueSend(display_messages_queue, &message, 1);
    }
}

Context *memory_lcd_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
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

static void display_init(Context *ctx, term opts)
{
#if ENABLE_INIT_SPI_BUS == true
    spi_display_bus_init();
#endif

    screen = malloc(sizeof(struct Screen));
    // FIXME: hardcoded width and height
    screen->w = 400;
    screen->h = 240;
    int memsize = 2 + 400 / 8 + 2;

    screen->pixels = heap_caps_malloc(memsize, MALLOC_CAP_DMA);
    if (UNLIKELY(!screen->pixels)) {
        fprintf(stderr, "failed to allocate buf!\n");
        abort();
    }

    screen->dma_out = heap_caps_malloc(memsize, MALLOC_CAP_DMA);
    if (UNLIKELY(!screen->dma_out)) {
        fprintf(stderr, "failed to allocate buf!\n");
        abort();
    }

    display_messages_queue = xQueueCreate(32, sizeof(Message *));

    GlobalContext *glb = ctx->global;
    struct ESP32PlatformData *platform = glb->platform_data;

    struct SPI *spi = malloc(sizeof(struct SPI));
    ctx->platform_data = spi;

    spi->ctx = ctx;
    spi->replies_queue = xQueueCreate(32, sizeof(struct PendingReply));
    spi->listener.sender = spi;
    spi->listener.data = spi;
    spi->listener.handler = display_callback;
    list_append(&platform->listeners, &spi->listener.listeners_list_head);

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.mode = 0;
    spi_config.clock_speed_hz = 1000000;
    spi_config.cs_active_high = true;
    spi_config.bit_lsb_first = true;
    spi_config.cs_ena_pretrans = 4; // it should be at least 3us
    spi_config.cs_ena_posttrans = 2; // it should be at least 1us
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&spi->spi_disp, &spi_config);

    int display_enable_gpio;
    bool ok = display_common_gpio_from_opts(
        opts, ATOM_STR("\x13", "display_enable_gpio"), &display_enable_gpio, glb);

    if (ok) {
        gpio_set_direction(display_enable_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(display_enable_gpio, 1);
    }

    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
}
