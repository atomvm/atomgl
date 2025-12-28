/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2022 Davide Bettio <davide@uninstall.it>
 * Copyright 2025 Masatoshi Nishiguchi
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

/* Based on ili934x_display_driver.c. */

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
#include "display_common.h"
#include "display_items.h"
#include "image_helpers.h"
#include "spi_display.h"

#define SPI_CLOCK_HZ 27000000
#define SPI_MODE 0

#define CHAR_WIDTH 8

#define ILI948X_SWRESET 0x01
#define ILI948X_SLPIN 0x10
#define ILI948X_SLPOUT 0x11
#define ILI948X_DISPOFF 0x28
#define ILI948X_DISPON 0x29

#define ILI948X_CASET 0x2A
#define ILI948X_PASET 0x2B
#define ILI948X_RAMWR 0x2C

#define ILI948X_MADCTL 0x36
#define ILI948X_MAD_MY 0x80
#define ILI948X_MAD_MX 0x40
#define ILI948X_MAD_MV 0x20
#define ILI948X_MAD_BGR 0x08

#define ILI948X_INVOFF 0x20
#define ILI948X_INVON 0x21

#define ILI948X_PIXFMT 0x3A

#define ILI948X_IFMODE 0xB0
#define ILI948X_FRMCTR1 0xB1
#define ILI948X_INVCTR 0xB4
#define ILI948X_DFUNCTR 0xB6
#define ILI948X_ETMOD 0xB7
#define ILI948X_PWRCTR1 0xC0
#define ILI948X_PWRCTR2 0xC1
#define ILI948X_PWRCTR3 0xC2
#define ILI948X_VMCTR1 0xC5
#define ILI948X_HS_LANES_CTRL 0xBE
#define ILI948X_IMAGE_FUNCTION 0xE9
#define ILI948X_PGAMCTRL 0xE0
#define ILI948X_NGAMCTRL 0xE1
#define ILI948X_DGAMCTRL 0xE2
#define ILI948X_ADJCTRL3 0xF7

#define ILI948X_TFTWIDTH 320
#define ILI948X_TFTHEIGHT 480

#define UNUSED(x) ((void) (x))

#include "font.c"

static const char *TAG = "ili948x_display_driver";

static void send_message(term pid, term message, GlobalContext *global);

struct SPI
{
    struct SPIDisplay spi_disp;
    int dc_gpio;
    int reset_gpio;

    avm_int_t rotation;
    bool is_ili9488;

    bool madctl_bgr;

    Context *ctx;
};

// Double-buffered scanline buffers.
struct Screen
{
    int w;
    int h;
    uint16_t *pixels;
    uint16_t *pixels_out;

    // ILI9488: 3 bytes/pixel.
    uint8_t *bytes;
    uint8_t *bytes_out;
};

static struct Screen *screen;

// Alpha blending for RGB565.
static inline uint16_t alpha_blend_rgb565(uint32_t fg, uint32_t bg, uint8_t alpha)
{
    alpha = (alpha + 4) >> 3;
    bg = (bg | (bg << 16)) & 0b00000111111000001111100000011111;
    fg = (fg | (fg << 16)) & 0b00000111111000001111100000011111;
    uint32_t result = ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
    return (uint16_t) ((result >> 16) | result);
}

static inline uint8_t rgba8888_get_alpha(uint32_t color)
{
    return color & 0xFF;
}

static inline uint16_t rgba8888_color_to_rgb565(struct Screen *s, uint32_t color)
{
    UNUSED(s);

    uint8_t r = color >> 24;
    uint8_t g = (color >> 16) & 0xFF;
    uint8_t b = (color >> 8) & 0xFF;

    return (((uint16_t) (r >> 3)) << 11) | (((uint16_t) (g >> 2)) << 5) | ((uint16_t) b >> 3);
}

static inline uint16_t rgb565_color_to_surface(struct Screen *s, uint16_t color16)
{
    UNUSED(s);

    return (uint16_t) SPI_SWAP_DATA_TX(color16, 16);
}

static inline uint16_t uint32_color_to_surface(struct Screen *s, uint32_t color)
{
    uint16_t color16 = rgba8888_color_to_rgb565(s, color);

    return rgb565_color_to_surface(s, color16);
}

// ILI9488 scanline conversion: RGB565 -> RGB888 bytes.
static inline void rgb565swapped_line_to_rgb888(uint8_t *dst, const uint16_t *src_swapped, int n_pixels)
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

struct PendingReply
{
    uint64_t pending_call_ref_ticks;
    term pending_call_pid;
};

static QueueHandle_t display_messages_queue;

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx);
static void display_init(Context *ctx, term opts);

static void display_init9486(struct SPI *spi);
static void display_init9488(struct SPI *spi);

static inline void writedata(struct SPI *spi, uint32_t data)
{
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->spi_disp, 8, data);
    spi_device_release_bus(spi->spi_disp.handle);
}

static inline void writecommand(struct SPI *spi, uint8_t command)
{
    gpio_set_level(spi->dc_gpio, 0);
    writedata(spi, command);
    gpio_set_level(spi->dc_gpio, 1);
}

static inline void set_screen_paint_area(struct SPI *spi, int x, int y, int width, int height)
{
    writecommand(spi, ILI948X_CASET);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->spi_disp, 32, (x << 16) | ((x + width) - 1));
    spi_device_release_bus(spi->spi_disp.handle);

    writecommand(spi, ILI948X_PASET);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->spi_disp, 32, (y << 16) | ((y + height) - 1));
    spi_device_release_bus(spi->spi_disp.handle);
}

static int draw_image_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = rgba8888_color_to_rgb565(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * width + (xpos - x);
    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        if (alpha == 0xFF) {
            uint16_t color = uint32_color_to_surface(screen, img_pixel);
            pixmem16[drawn_pixels] = color;
        } else if (visible_bg) {
            uint16_t color = rgba8888_color_to_rgb565(screen, img_pixel);
            uint16_t blended = alpha_blend_rgb565(color, bgcolor, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(screen, blended);
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

static int draw_scaled_cropped_img_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = rgba8888_color_to_rgb565(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data_with_size.pix;

    int drawn_pixels = 0;

    int y_scale = item->y_scale;
    int x_scale = item->x_scale;
    int img_width = item->data.image_data_with_size.width;

    int source_x = item->source_x;
    int source_y = item->source_y;

    uint32_t *pixels = ((uint32_t *) data) + (source_y + ((ypos - y) / y_scale)) * img_width + source_x + ((xpos - x) / x_scale);
    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (source_x + (width / x_scale) > img_width) {
        width = (img_width - source_x) * x_scale;
    }

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        if (alpha == 0xFF) {
            uint16_t color = uint32_color_to_surface(screen, img_pixel);
            pixmem16[drawn_pixels] = color;
        } else if (visible_bg) {
            uint16_t color = rgba8888_color_to_rgb565(screen, img_pixel);
            uint16_t blended = alpha_blend_rgb565(color, bgcolor, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(screen, blended);
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels = ((uint32_t *) data) + (source_y + ((ypos - y) / y_scale)) * img_width + source_x + (j / x_scale);
    }

    return drawn_pixels;
}

static int draw_rect_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int width = item->width;
    uint16_t color = uint32_color_to_surface(screen, item->brcolor);

    int drawn_pixels = 0;

    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        pixmem16[drawn_pixels] = color;
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int draw_text_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;
    uint16_t fgcolor = uint32_color_to_surface(screen, item->data.text_data.fgcolor);
    uint16_t bgcolor;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = uint32_color_to_surface(screen, item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    char *text = (char *) item->data.text_data.text;

    int width = item->width;

    int drawn_pixels = 0;

    uint16_t *pixmem32 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

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
            pixmem32[drawn_pixels] = fgcolor;
        } else if (visible_bg) {
            pixmem32[drawn_pixels] = bgcolor;
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int find_max_line_len(BaseDisplayItem *items, int count, int xpos, int ypos)
{
    int line_len = screen->w - xpos;

    for (int i = 0; i < count; i++) {
        BaseDisplayItem *item = &items[i];

        if ((xpos < item->x) && (ypos >= item->y) && (ypos < item->y + item->height)) {
            int len_to_item = item->x - xpos;
            line_len = (line_len > len_to_item) ? len_to_item : line_len;
        }
    }

    return line_len;
}

static int draw_x(int xpos, int ypos, BaseDisplayItem *items, int items_count)
{
    bool below = false;

    for (int i = 0; i < items_count; i++) {
        BaseDisplayItem *item = &items[i];
        if ((xpos < item->x) || (xpos >= item->x + item->width) || (ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }

        int max_line_len = below ? 1 : find_max_line_len(items, i, xpos, ypos);

        int drawn_pixels = 0;
        switch (items[i].primitive) {
            case Image:
                drawn_pixels = draw_image_x(xpos, ypos, max_line_len, item);
                break;

            case Rect:
                drawn_pixels = draw_rect_x(xpos, ypos, max_line_len, item);
                break;

            case ScaledCroppedImage:
                drawn_pixels = draw_scaled_cropped_img_x(xpos, ypos, max_line_len, item);
                break;

            case Text:
                drawn_pixels = draw_text_x(xpos, ypos, max_line_len, item);
                break;
            default:
                fprintf(stderr, "unexpected display list command.\n");
                break;
        }

        if (drawn_pixels != 0) {
            return drawn_pixels;
        }

        below = true;
    }

    return 1;
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

    set_screen_paint_area(spi, 0, 0, screen_width, screen_height);
    writecommand(spi, ILI948X_RAMWR);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);

    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        if (transaction_in_progress) {
            spi_transaction_t *trans;
            spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
        }

        // Swap scanline buffers.
        void *tmp = screen->pixels;
        screen->pixels = screen->pixels_out;
        screen->pixels_out = tmp;

        if (!spi->is_ili9488) {
            spi_display_dmawrite(&spi->spi_disp, screen_width * sizeof(uint16_t), screen->pixels_out);
        } else {
            void *tmpb = screen->bytes;
            screen->bytes = screen->bytes_out;
            screen->bytes_out = tmpb;

            rgb565swapped_line_to_rgb888(screen->bytes_out, screen->pixels_out, screen_width);
            spi_display_dmawrite(&spi->spi_disp, screen_width * 3, screen->bytes_out);
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

static void draw_buffer(struct SPI *spi, int x, int y, int width, int height, const void *imgdata)
{
    const uint16_t *data = imgdata;

    set_screen_paint_area(spi, x, y, width, height);

    writecommand(spi, ILI948X_RAMWR);

    int dest_size = width * height;
    int chunks = dest_size / 1024;

    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);

    if (!spi->is_ili9488) {
        int buf_pixel_size = (dest_size > 1024) ? 1024 : dest_size;
        uint16_t *tmpbuf = heap_caps_malloc(buf_pixel_size * sizeof(uint16_t), MALLOC_CAP_DMA);

        for (int i = 0; i < chunks; i++) {
            const uint16_t *data_b = data + 1024 * i;
            for (int j = 0; j < 1024; j++) {
                tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
            }
            spi_display_dmawrite(&spi->spi_disp, 1024 * sizeof(uint16_t), tmpbuf);
        }

        int last_chunk_size = dest_size - chunks * 1024;
        if (last_chunk_size) {
            const uint16_t *data_b = data + chunks * 1024;
            for (int j = 0; j < last_chunk_size; j++) {
                tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
            }
            spi_display_dmawrite(&spi->spi_disp, last_chunk_size * sizeof(uint16_t), tmpbuf);
        }

        free(tmpbuf);

    } else {
        // ILI9488: RGB565 -> RGB888 (3 bytes/pixel).
        const int chunk_pixels = 512;
        uint8_t *tmpbuf = heap_caps_malloc(chunk_pixels * 3, MALLOC_CAP_DMA);

        int i = 0;
        while (i < dest_size) {
            int n = (dest_size - i > chunk_pixels) ? chunk_pixels : (dest_size - i);

            for (int j = 0; j < n; j++) {
                uint16_t px = data[i + j];
                uint8_t r5 = (px >> 11) & 0x1F;
                uint8_t g6 = (px >> 5) & 0x3F;
                uint8_t b5 = (px >> 0) & 0x1F;

                uint8_t r8 = (r5 << 3) | (r5 >> 2);
                uint8_t g8 = (g6 << 2) | (g6 >> 4);
                uint8_t b8 = (b5 << 3) | (b5 >> 2);

                tmpbuf[j * 3 + 0] = r8;
                tmpbuf[j * 3 + 1] = g8;
                tmpbuf[j * 3 + 2] = b8;
            }

            spi_display_dmawrite(&spi->spi_disp, n * 3, tmpbuf);
            i += n;
        }

        free(tmpbuf);
    }

    spi_device_release_bus(spi->spi_disp.handle);
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

    struct SPI *spi = ctx->platform_data;

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {
        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);

    } else if (cmd == context_make_atom(ctx, "\xB"
                                             "draw_buffer")) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        unsigned long addr_low = term_to_int(term_get_tuple_element(req, 5));
        unsigned long addr_high = term_to_int(term_get_tuple_element(req, 6));

        const void *data = (const void *) ((addr_low | (addr_high << 16)));

        draw_buffer(spi, x, y, width, height, data);

        // draw_buffer is fire-and-forget.
        return;

    } else if (cmd == globalcontext_make_atom(ctx->global, "\xA"
                                                           "load_image")) {
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

    send_message(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void process_messages(void *arg)
{
    struct SPI *args = arg;

    while (true) {
        Message *message;
        xQueueReceive(display_messages_queue, &message, portMAX_DELAY);
        process_message(message, args->ctx);

        BEGIN_WITH_STACK_HEAP(1, temp_heap);
        mailbox_message_dispose(&message->base, &temp_heap);
        END_WITH_STACK_HEAP(temp_heap, args->ctx->global);
    }
}

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx)
{
    MailboxMessage *mbox_msg = mailbox_take_message(&ctx->mailbox);
    Message *msg = CONTAINER_OF(mbox_msg, Message, base);

    // Non-blocking enqueue; drop oldest on overflow.
    if (xQueueSend(display_messages_queue, &msg, 0) != pdTRUE) {

        Message *old = NULL;
        if (xQueueReceive(display_messages_queue, &old, 0) == pdTRUE && old) {
            BEGIN_WITH_STACK_HEAP(1, temp_heap);
            mailbox_message_dispose(&old->base, &temp_heap);
            END_WITH_STACK_HEAP(temp_heap, ctx->global);
        }

        if (xQueueSend(display_messages_queue, &msg, 0) != pdTRUE) {
            BEGIN_WITH_STACK_HEAP(1, temp_heap2);
            mailbox_message_dispose(&msg->base, &temp_heap2);
            END_WITH_STACK_HEAP(temp_heap2, ctx->global);
        }
    }

    return NativeContinue;
}

static void set_rotation(struct SPI *spi, int rotation)
{
    uint8_t madctl = 0;

    if (spi->madctl_bgr) {
        madctl |= ILI948X_MAD_BGR;
    }

    switch (rotation & 3) {
        case 0:
            madctl |= ILI948X_MAD_MX;
            break;

        case 1:
            madctl |= ILI948X_MAD_MV;
            break;

        case 2:
            madctl |= ILI948X_MAD_MY;
            break;

        case 3:
            madctl |= ILI948X_MAD_MX | ILI948X_MAD_MY | ILI948X_MAD_MV;
            break;
    }

    writecommand(spi, ILI948X_MADCTL);
    writedata(spi, madctl);
}

Context *ili948x_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
}

static void send_message(term pid, term message, GlobalContext *global)
{
    int local_process_id = term_to_local_process_id(pid);
    globalcontext_send_message(global, local_process_id, message);
}

static void display_init(Context *ctx, term opts)
{
    screen = malloc(sizeof(struct Screen));

    display_messages_queue = xQueueCreate(32, sizeof(Message *));

    struct SPI *spi = malloc(sizeof(struct SPI));
    ctx->platform_data = spi;

    spi->ctx = ctx;

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.mode = SPI_MODE;
    spi_config.clock_speed_hz = SPI_CLOCK_HZ;
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&spi->spi_disp, &spi_config);

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &spi->dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &spi->reset_gpio, ctx->global);

    term compat_value_term = interop_kv_get_value_default(opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_value_term, &str_ok);

    bool is_ili9486 = false;
    bool is_ili9488 = false;
    if (str_ok && compat_string) {
        is_ili9486 = !strcmp(compat_string, "ilitek,ili9486");
        is_ili9488 = !strcmp(compat_string, "ilitek,ili9488");
        free(compat_string);
    } else {
        ok = false;
    }

    if (!is_ili9486 && !is_ili9488) {
        ok = false;
    }
    spi->is_ili9488 = is_ili9488;

    // color_order: rgb|bgr (default: bgr)
    term color_order_term = interop_kv_get_value_default(opts, ATOM_STR("\xB", "color_order"), term_nil(), ctx->global);

    if (term_is_nil(color_order_term)) {
        spi->madctl_bgr = true;
    } else if (term_is_atom(color_order_term)) {
        if (color_order_term == context_make_atom(ctx, "\x3"
                                                       "rgb")) {
            spi->madctl_bgr = false;
        } else if (color_order_term == context_make_atom(ctx, "\x3"
                                                              "bgr")) {
            spi->madctl_bgr = true;
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }

    term rotation = interop_kv_get_value_default(opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);
    ok = ok && term_is_integer(rotation);
    spi->rotation = term_to_int(rotation);

    term invon = interop_kv_get_value_default(opts, ATOM_STR("\x10", "enable_tft_invon"), FALSE_ATOM, ctx->global);
    ok = ok && ((invon == TRUE_ATOM) || (invon == FALSE_ATOM));
    bool enable_tft_invon = (invon == TRUE_ATOM);

    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display parameters.");
        return;
    }

    // Swap w/h for 90/270.
    if (spi->rotation & 1) {
        screen->w = ILI948X_TFTHEIGHT;
        screen->h = ILI948X_TFTWIDTH;
    } else {
        screen->w = ILI948X_TFTWIDTH;
        screen->h = ILI948X_TFTHEIGHT;
    }

    screen->pixels = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);
    screen->pixels_out = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);

    if (spi->is_ili9488) {
        screen->bytes = heap_caps_malloc(screen->w * 3, MALLOC_CAP_DMA);
        screen->bytes_out = heap_caps_malloc(screen->w * 3, MALLOC_CAP_DMA);
    } else {
        screen->bytes = NULL;
        screen->bytes_out = NULL;
    }

    // Reset.
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    gpio_set_direction(spi->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(spi->reset_gpio, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(spi->reset_gpio, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(spi->reset_gpio, 1);
    spi_device_release_bus(spi->spi_disp.handle);

    gpio_set_direction(spi->dc_gpio, GPIO_MODE_OUTPUT);

    writecommand(spi, ILI948X_SWRESET);

    vTaskDelay(5 / portTICK_PERIOD_MS);

    if (spi->is_ili9488) {
        display_init9488(spi);
    } else {
        display_init9486(spi);
    }

    writecommand(spi, ILI948X_SLPOUT);

    vTaskDelay(120 / portTICK_PERIOD_MS);

    writecommand(spi, ILI948X_DISPON);

    if (enable_tft_invon) {
        writecommand(spi, ILI948X_INVON);
    } else {
        writecommand(spi, ILI948X_INVOFF);
    }

    set_rotation(spi, spi->rotation);

    struct BacklightGPIOConfig backlight_config;
    backlight_gpio_init_config(&backlight_config);
    backlight_gpio_parse_config(&backlight_config, opts, ctx->global);
    backlight_gpio_init(&backlight_config);

    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
}

static void display_init9486(struct SPI *spi)
{
    writecommand(spi, ILI948X_IFMODE);
    writedata(spi, 0x00);

    writecommand(spi, ILI948X_PIXFMT);
    writedata(spi, 0x55);

    writecommand(spi, ILI948X_PWRCTR3);
    writedata(spi, 0x44);

    writecommand(spi, ILI948X_VMCTR1);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x00);

    writecommand(spi, ILI948X_PGAMCTRL);
    writedata(spi, 0x0f);
    writedata(spi, 0x1f);
    writedata(spi, 0x1c);
    writedata(spi, 0x0c);
    writedata(spi, 0x0f);
    writedata(spi, 0x08);
    writedata(spi, 0x48);
    writedata(spi, 0x98);
    writedata(spi, 0x37);
    writedata(spi, 0x0a);
    writedata(spi, 0x13);
    writedata(spi, 0x04);
    writedata(spi, 0x11);
    writedata(spi, 0x0d);
    writedata(spi, 0x00);

    writecommand(spi, ILI948X_NGAMCTRL);
    writedata(spi, 0x0f);
    writedata(spi, 0x32);
    writedata(spi, 0x2e);
    writedata(spi, 0x0b);
    writedata(spi, 0x0d);
    writedata(spi, 0x05);
    writedata(spi, 0x47);
    writedata(spi, 0x75);
    writedata(spi, 0x37);
    writedata(spi, 0x06);
    writedata(spi, 0x10);
    writedata(spi, 0x03);
    writedata(spi, 0x24);
    writedata(spi, 0x20);
    writedata(spi, 0x00);

    writecommand(spi, ILI948X_DGAMCTRL);
    writedata(spi, 0x0f);
    writedata(spi, 0x32);
    writedata(spi, 0x2e);
    writedata(spi, 0x0b);
    writedata(spi, 0x0d);
    writedata(spi, 0x05);
    writedata(spi, 0x47);
    writedata(spi, 0x75);
    writedata(spi, 0x37);
    writedata(spi, 0x06);
    writedata(spi, 0x10);
    writedata(spi, 0x03);
    writedata(spi, 0x24);
    writedata(spi, 0x20);
    writedata(spi, 0x00);
}

static void display_init9488(struct SPI *spi)
{
    // ILI9488: RGB666 over SPI (3 bytes/pixel).
    writecommand(spi, ILI948X_IFMODE);
    writedata(spi, 0x00);

    writecommand(spi, ILI948X_ADJCTRL3);
    writedata(spi, 0xA9);
    writedata(spi, 0x51);
    writedata(spi, 0x2C);
    writedata(spi, 0x82);

    writecommand(spi, ILI948X_PWRCTR1);
    writedata(spi, 0x11);
    writedata(spi, 0x09);

    writecommand(spi, ILI948X_PWRCTR2);
    writedata(spi, 0x41);

    writecommand(spi, ILI948X_VMCTR1);
    writedata(spi, 0x00);
    writedata(spi, 0x0A);
    writedata(spi, 0x80);

    writecommand(spi, ILI948X_FRMCTR1);
    writedata(spi, 0xB0);
    writedata(spi, 0x11);

    writecommand(spi, ILI948X_INVCTR);
    writedata(spi, 0x02);

    writecommand(spi, ILI948X_DFUNCTR);
    writedata(spi, 0x02);
    writedata(spi, 0x02);

    writecommand(spi, ILI948X_ETMOD);
    writedata(spi, 0xC6);

    writecommand(spi, ILI948X_HS_LANES_CTRL);
    writedata(spi, 0x00);
    writedata(spi, 0x04);

    writecommand(spi, ILI948X_IMAGE_FUNCTION);
    writedata(spi, 0x00);

    writecommand(spi, ILI948X_PIXFMT);
    writedata(spi, 0x66);

    writecommand(spi, ILI948X_PGAMCTRL);
    writedata(spi, 0x00);
    writedata(spi, 0x07);
    writedata(spi, 0x10);
    writedata(spi, 0x09);
    writedata(spi, 0x17);
    writedata(spi, 0x0B);
    writedata(spi, 0x41);
    writedata(spi, 0x89);
    writedata(spi, 0x4B);
    writedata(spi, 0x0A);
    writedata(spi, 0x0C);
    writedata(spi, 0x0E);
    writedata(spi, 0x18);
    writedata(spi, 0x1B);
    writedata(spi, 0x0F);

    writecommand(spi, ILI948X_NGAMCTRL);
    writedata(spi, 0x00);
    writedata(spi, 0x17);
    writedata(spi, 0x1A);
    writedata(spi, 0x04);
    writedata(spi, 0x0E);
    writedata(spi, 0x06);
    writedata(spi, 0x2F);
    writedata(spi, 0x45);
    writedata(spi, 0x43);
    writedata(spi, 0x02);
    writedata(spi, 0x0A);
    writedata(spi, 0x09);
    writedata(spi, 0x32);
    writedata(spi, 0x36);
    writedata(spi, 0x0F);
}
