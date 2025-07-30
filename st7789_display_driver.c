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
#include "display_common.h"
#include "display_items.h"
#include "image_helpers.h"
#include "spi_display.h"

// if needed it can be lowered to 27000000, while maximum is 62.5 Mhz
#define SPI_CLOCK_HZ 40000000
#define SPI_MODE 0

#define CHAR_WIDTH 8

#define ST7789_SWRESET 0x01
#define ST7789_SLPIN 0x10
#define ST7789_SLPOUT 0x11
#define ST7789_NORON 0x13
#define ST7789_INVON 0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON 0x29
#define ST7789_CASET 0x2A
#define ST7789_RASET 0x2B
#define ST7789_RAMWR 0x2C
#define ST7789_MADCTL 0x36
#define ST7789_COLMOD 0x3A
#define ST7789_RAMCTRL 0xB0
#define ST7789_PORCTRL 0xB2
#define ST7789_GCTRL 0xB7
#define ST7789_VCOMS 0xBB
#define ST7789_LCMCTRL 0xC0
#define ST7789_VDVVRHEN 0xC2
#define ST7789_VRHS 0xC3
#define ST7789_VDVSET 0xC4
#define ST7789_FRCTR2 0xC6
#define ST7789_PWCTRL1 0xD0
#define ST7789_PVGAMCTRL 0xE0
#define ST7789_NVGAMCTRL 0xE1

// rotation
#define ST7789_MADCTL_MY 0x80
#define ST7789_MADCTL_MX 0x40
#define ST7789_MADCTL_MV 0x20
#define ST7789_MADCTL_ML 0x10
#define ST7789_MADCTL_RGB 0x00

#define TFT_MAD_RGB 0x00
#define TFT_MAD_BGR 0x08
#define TFT_MAD_COLOR_ORDER TFT_MAD_RGB

#include "font.c"

static const char *TAG = "st7789_display_driver";

static void send_message(term pid, term message, GlobalContext *global);

static inline void delay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

struct SPI
{
    struct SPIDisplay spi_disp;
    int dc_gpio;
    int reset_gpio;

    avm_int_t rotation;

    Context *ctx;
};

// This struct is just for compatibility reasons with the SDL display driver
// so it is possible to easily copy & paste code from there.
struct Screen
{
    int w;
    int h;
    avm_int_t x_offset;
    avm_int_t y_offset;
    uint16_t *pixels;
    uint16_t *pixels_out;
};

static struct Screen *screen;

// This functions is taken from:
// https://stackoverflow.com/questions/18937701/combining-two-16-bits-rgb-colors-with-alpha-blending
static inline uint16_t alpha_blend_rgb565(uint32_t fg, uint32_t bg, uint8_t alpha)
{
    alpha = (alpha + 4) >> 3;
    bg = (bg | (bg << 16)) & 0b00000111111000001111100000011111;
    fg = (fg | (fg << 16)) & 0b00000111111000001111100000011111;
    uint32_t result = ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
    return (uint16_t)((result >> 16) | result);
}

static inline uint8_t rgba8888_get_alpha(uint32_t color)
{
    return color & 0xFF;
}

static inline uint16_t rgba8888_color_to_rgb565(struct Screen *s, uint32_t color)
{
    uint8_t r = color >> 24;
    uint8_t g = (color >> 16) & 0xFF;
    uint8_t b = (color >> 8) & 0xFF;

    return (((uint16_t)(r >> 3)) << 11) | (((uint16_t)(g >> 2)) << 5) | ((uint16_t) b >> 3);
}

static inline uint16_t rgb565_color_to_surface(struct Screen *s, uint16_t color16)
{
    return (uint16_t) SPI_SWAP_DATA_TX(color16, 16);
}

static inline uint16_t uint32_color_to_surface(struct Screen *s, uint32_t color)
{
    uint16_t color16 = rgba8888_color_to_rgb565(s, color);

    return rgb565_color_to_surface(s, color16);
}

struct PendingReply
{
    uint64_t pending_call_ref_ticks;
    term pending_call_pid;
};

static QueueHandle_t display_messages_queue;

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx);
static void display_init(Context *ctx, term opts);
static void display_init_alt_gamma_2(struct SPI *spi);
static void display_init_std(struct SPI *spi);
static void display_init_using_list(struct SPI *spi, term init_list);

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
    x += screen->x_offset;
    y += screen->y_offset;

    writecommand(spi, ST7789_CASET);
    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->spi_disp, 32, (x << 16) | ((x + width) - 1));
    spi_device_release_bus(spi->spi_disp.handle);

    writecommand(spi, ST7789_RASET);
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
        // TODO: optimize here
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
            default: {
                fprintf(stderr, "unexpected display list command.\n");
            }
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
    writecommand(spi, ST7789_RAMWR);
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
            // I did a quick measurement, and most of the time is spent waiting for DMA transaction
            // eg. 23 us spent in draw_x, 188 us spent in spi_device_get_trans_result
            spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
        }

        // NEW CODE
        void *tmp = screen->pixels;
        screen->pixels = screen->pixels_out;
        screen->pixels_out = tmp;
        spi_display_dmawrite(&spi->spi_disp, screen_width * sizeof(uint16_t), screen->pixels_out);
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

    writecommand(spi, ST7789_RAMWR);

    int dest_size = width * height;
    int buf_pixel_size = (dest_size > 1024) ? 1024 : dest_size;

    int chunks = dest_size / 1024;

    uint16_t *tmpbuf = heap_caps_malloc(buf_pixel_size * sizeof(uint16_t), MALLOC_CAP_DMA);

    spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    for (int i = 0; i < chunks; i++) {
        const uint16_t *data_b = data + 1024 * i;
        for (int j = 0; j < 1024; j++) {
            tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
        }
        spi_display_dmawrite(&spi->spi_disp, buf_pixel_size * sizeof(uint16_t), tmpbuf);
    }
    int last_chunk_size = dest_size - chunks * 1024;
    if (last_chunk_size) {
        const uint16_t *data_b = data + chunks * 1024;
        for (int j = 0; j < 1024; j++) {
            tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
        }
        spi_display_dmawrite(&spi->spi_disp, last_chunk_size * sizeof(uint16_t), tmpbuf);
    }
    spi_device_release_bus(spi->spi_disp.handle);

    free(tmpbuf);
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

        // draw_buffer is a kind of cast, no need to reply
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

    xQueueSend(display_messages_queue, &msg, 1);

    return NativeContinue;
}

static void set_rotation(struct SPI *spi, int rotation)
{
    if (rotation == 1) {
        writecommand(spi, ST7789_MADCTL);
        writedata(spi, ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
    }
}

Context *st7789_display_create_port(GlobalContext *global, term opts)
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
    term width_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x5", "width"), term_from_int(320), ctx->global);
    term height_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x6", "height"), term_from_int(240), ctx->global);

    screen = malloc(sizeof(struct Screen));
    screen->w = term_to_int(width_term);
    screen->h = term_to_int(height_term);
    screen->pixels = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);
    screen->pixels_out = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);

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

    bool reset_configured = true;
    if (!display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &spi->reset_gpio, ctx->global)) {
        ESP_LOGI(TAG, "Reset GPIO not configured.");
        reset_configured = false;
    }

    term rotation = interop_kv_get_value_default(opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);
    ok = ok && term_is_integer(rotation);
    spi->rotation = term_to_int(rotation);

    term invon = interop_kv_get_value_default(opts, ATOM_STR("\x10", "enable_tft_invon"), FALSE_ATOM, ctx->global);
    ok = ok && ((invon == TRUE_ATOM) || (invon == FALSE_ATOM));
    bool enable_tft_invon = (invon == TRUE_ATOM);

    term x_off_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x8", "x_offset"), term_from_int(0), ctx->global);
    term y_off_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x8", "y_offset"), term_from_int(0), ctx->global);

    if (term_is_integer(x_off_term) && term_is_integer(y_off_term)) {
        screen->x_offset = term_to_int(x_off_term);
        screen->y_offset = term_to_int(y_off_term);
    } else {
        ok = false;
    }

    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display parameters.");
        return;
    }

    // Reset
    if (reset_configured) {
        spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
        gpio_set_direction(spi->reset_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(spi->reset_gpio, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(spi->reset_gpio, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(spi->reset_gpio, 1);
        spi_device_release_bus(spi->spi_disp.handle);
    }

    gpio_set_direction(spi->dc_gpio, GPIO_MODE_OUTPUT);

    if (!reset_configured) {
        writecommand(spi, ST7789_SWRESET);
        delay(100);
    }

    term maybe_init_list
        = interop_kv_get_value_default(opts, ATOM_STR("\x9", "init_list"), term_nil(), ctx->global);
    if (maybe_init_list != term_nil()) {
        display_init_using_list(spi, maybe_init_list);
    } else {
        term init_seq_type_term = interop_kv_get_value_default(opts, ATOM_STR("\xD", "init_seq_type"), term_nil(), ctx->global);
        int str_ok;
        char *init_seq_type_string = interop_term_to_string(init_seq_type_term, &str_ok);
        if (str_ok && !strcmp(init_seq_type_string, "alt_gamma_2")) {
            display_init_alt_gamma_2(spi);
            free(init_seq_type_string);
        } else {
            display_init_std(spi);
        }

        set_rotation(spi, spi->rotation);

        if (enable_tft_invon) {
            writecommand(spi, ST7789_INVON);
        }
    }

    writecommand(spi, ST7789_DISPON);
    delay(120);

    struct BacklightGPIOConfig backlight_config;
    backlight_gpio_init_config(&backlight_config);
    backlight_gpio_parse_config(&backlight_config, opts, ctx->global);
    backlight_gpio_init(&backlight_config);

    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
}

static void display_init_alt_gamma_2(struct SPI *spi)
{
    writecommand(spi, ST7789_SLPOUT);
    delay(120);

    writecommand(spi, ST7789_NORON);

    // - display and color format setting - //
    writecommand(spi, ST7789_MADCTL);
    writedata(spi, TFT_MAD_COLOR_ORDER);

    writecommand(spi, ST7789_COLMOD);
    writedata(spi, 0x55);
    delay(10);

    // - ST7789V frame rate setting - //
    writecommand(spi, ST7789_PORCTRL);
    writedata(spi, 0x0C);
    writedata(spi, 0x0C);
    writedata(spi, 0x00);
    writedata(spi, 0x33);
    writedata(spi, 0x33);

    writecommand(spi, ST7789_GCTRL);
    writedata(spi, 0x75);

    // - ST7789V power setting - //
    writecommand(spi, ST7789_VCOMS);
    writedata(spi, 0x1A);

    writecommand(spi, ST7789_LCMCTRL);
    writedata(spi, 0x2C);

    writecommand(spi, ST7789_VDVVRHEN);
    writedata(spi, 0x01);

    writecommand(spi, ST7789_VRHS);
    writedata(spi, 0x13);

    writecommand(spi, ST7789_VDVSET);
    writedata(spi, 0x20);

    writecommand(spi, ST7789_FRCTR2);
    writedata(spi, 0x0F);

    writecommand(spi, ST7789_PWCTRL1);
    writedata(spi, 0xA4);
    writedata(spi, 0xA1);

    // - ST7789V gamma setting - //
    writecommand(spi, ST7789_PVGAMCTRL);
    writedata(spi, 0xD0);
    writedata(spi, 0x0D);
    writedata(spi, 0x14);
    writedata(spi, 0x0D);
    writedata(spi, 0x0D);
    writedata(spi, 0x09);
    writedata(spi, 0x38);
    writedata(spi, 0x44);
    writedata(spi, 0x4E);
    writedata(spi, 0x3A);
    writedata(spi, 0x17);
    writedata(spi, 0x18);
    writedata(spi, 0x2F);
    writedata(spi, 0x30);

    writecommand(spi, ST7789_NVGAMCTRL);
    writedata(spi, 0xD0);
    writedata(spi, 0x09);
    writedata(spi, 0x0F);
    writedata(spi, 0x08);
    writedata(spi, 0x07);
    writedata(spi, 0x14);
    writedata(spi, 0x37);
    writedata(spi, 0x44);
    writedata(spi, 0x4D);
    writedata(spi, 0x38);
    writedata(spi, 0x15);
    writedata(spi, 0x16);
    writedata(spi, 0x2C);
    writedata(spi, 0x3E);

    writecommand(spi, ST7789_CASET);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0xEF); // 239

    writecommand(spi, ST7789_RASET);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x01);
    writedata(spi, 0x3F); // 319
}

static void display_init_std(struct SPI *spi)
{
    writecommand(spi, ST7789_SLPOUT);
    delay(120);

    writecommand(spi, ST7789_NORON);

    // - display and color format setting - //
    writecommand(spi, ST7789_MADCTL);
    writedata(spi, TFT_MAD_COLOR_ORDER);

    writecommand(spi, 0xB6);
    writedata(spi, 0x0A);
    writedata(spi, 0x82);

    writecommand(spi, ST7789_RAMCTRL);
    writedata(spi, 0x00);
    writedata(spi, 0xE0);

    writecommand(spi, ST7789_COLMOD);
    writedata(spi, 0x55);
    delay(10);

    // - ST7789V frame rate setting - //
    writecommand(spi, ST7789_PORCTRL);
    writedata(spi, 0x0C);
    writedata(spi, 0x0C);
    writedata(spi, 0x00);
    writedata(spi, 0x33);
    writedata(spi, 0x33);

    writecommand(spi, ST7789_GCTRL);
    writedata(spi, 0x35);

    // - ST7789V power setting - //
    writecommand(spi, ST7789_VCOMS);
    writedata(spi, 0x28);

    writecommand(spi, ST7789_LCMCTRL);
    writedata(spi, 0x0C);

    writecommand(spi, ST7789_VDVVRHEN);
    writedata(spi, 0x01);
    writedata(spi, 0xFF);

    writecommand(spi, ST7789_VRHS);
    writedata(spi, 0x10);

    writecommand(spi, ST7789_VDVSET);
    writedata(spi, 0x20);

    writecommand(spi, ST7789_FRCTR2);
    writedata(spi, 0x0F);

    writecommand(spi, ST7789_PWCTRL1);
    writedata(spi, 0xA4);
    writedata(spi, 0xA1);

    // - ST7789V gamma setting - //
    writecommand(spi, ST7789_PVGAMCTRL);
    writedata(spi, 0xD0);
    writedata(spi, 0x00);
    writedata(spi, 0x02);
    writedata(spi, 0x07);
    writedata(spi, 0x0A);
    writedata(spi, 0x28);
    writedata(spi, 0x32);
    writedata(spi, 0x44);
    writedata(spi, 0x42);
    writedata(spi, 0x06);
    writedata(spi, 0x0E);
    writedata(spi, 0x12);
    writedata(spi, 0x14);
    writedata(spi, 0x17);

    writecommand(spi, ST7789_NVGAMCTRL);
    writedata(spi, 0xD0);
    writedata(spi, 0x00);
    writedata(spi, 0x02);
    writedata(spi, 0x07);
    writedata(spi, 0x0A);
    writedata(spi, 0x28);
    writedata(spi, 0x31);
    writedata(spi, 0x54);
    writedata(spi, 0x47);
    writedata(spi, 0x0E);
    writedata(spi, 0x1C);
    writedata(spi, 0x17);
    writedata(spi, 0x1B);
    writedata(spi, 0x1E);

    writecommand(spi, ST7789_CASET);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0xEF); // 239

    writecommand(spi, ST7789_RASET);
    writedata(spi, 0x00);
    writedata(spi, 0x00);
    writedata(spi, 0x01);
    writedata(spi, 0x3F); // 319
}

static void writecmddata(struct SPI *spi, uint8_t cmd, const uint8_t *data, size_t length)
{
    writecommand(spi, cmd);
    for (int i = 0; i < length; i++) {
        writedata(spi, data[i]);
    }
}

static void display_init_using_list(struct SPI *spi, term init_list)
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
                writecmddata(spi, cmd, data, term_binary_size(data_term));
            } else if ((cmd_term == context_make_atom(spi->ctx, ATOM_STR("\x8", "sleep_ms")))
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
