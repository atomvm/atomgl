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

#define DC_IO_NUM CONFIG_AVM_ILI934X_DC_IO_NUM

#define MISO_IO_NUM CONFIG_AVM_ILI934X_MISO_IO_NUM
#define MOSI_IO_NUM CONFIG_AVM_ILI934X_MOSI_IO_NUM
#define SCLK_IO_NUM CONFIG_AVM_ILI934X_SCLK_IO_NUM
#define SPI_CLOCK_HZ CONFIG_AVM_ILI934X_SPI_CLOCK_HZ
#define DISPLAY_CS_IO_NUM CONFIG_AVM_ILI934X_DISPLAY_CS_IO_NUM
#define SPI_MODE 0
#define ADDRESS_LEN_BITS 0

#define SD_ENABLE CONFIG_AVM_ILI934X_SD_ENABLE
#define SD_CS_IO_NUM CONFIG_AVM_ILI934X_SD_CS_IO_NUM

#define CHAR_WIDTH 8

#define DISPLAY_WIDTH 400

#define CHECK_OVERFLOW 1
#define REPORT_UNEXPECTED_MSGS 0

#include "font.c"

struct SPI
{
    spi_device_handle_t handle;
    spi_transaction_t transaction;
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
static void display_init(Context *ctx, int cs_gpio);

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
    const uint8_t m[4][4] = {
        { 0, 8, 2, 10 },
        { 12, 4, 14, 6 },
        { 3, 11, 1, 9 },
        { 15, 7, 13, 5 }
    };

    // r = 255 / values / 4
    float out_r = r + roundf(63.75 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    float out_g = g + roundf(63.75 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    float out_b = b + roundf(63.75 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    // end of dither

    // get closest
    float yval = 0.2126 * out_r + 0.7152 * out_g + 0.0722 * out_b;
    return yval >= 128;
}

#if SD_ENABLE == true
// sdcard init in display driver is quite an odd choice
// however display and SD share the same bus
static bool sdcard_init()
{
    ESP_LOGI("sdcard", "Trying SD init.");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = MISO_IO_NUM;
    slot_config.gpio_mosi = MOSI_IO_NUM;
    slot_config.gpio_sck = SCLK_IO_NUM;
    slot_config.gpio_cs = SD_CS_IO_NUM;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("sdcard", "Failed to mount filesystem.");
        } else {
            ESP_LOGE("sdcard", "Failed to initialize the card (%s).", esp_err_to_name(ret));
        }
        return false;
    }

    sdmmc_card_print_info(stdout, card);

    return true;
}
#endif

static bool spidmawrite(struct SPI *spi_data, int data_len, const void *data)
{
    memset(&spi_data->transaction, 0, sizeof(spi_transaction_t));

    spi_data->transaction.flags = 0;
    spi_data->transaction.length = data_len * 8;
    spi_data->transaction.addr = 0;
    spi_data->transaction.tx_buffer = data;

    int ret = spi_device_queue_trans(spi_data->handle, &spi_data->transaction, portMAX_DELAY);
    if (UNLIKELY(ret != ESP_OK)) {
        fprintf(stderr, "spidmawrite: transmit error\n");
        return false;
    }

    return true;
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
    line_buf[xpos / 8] = (line_buf[xpos / 8] & ~(0x1 << bpos)) | ((color & 0x1) << bpos);
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

    spi_device_acquire_bus(spi->handle, portMAX_DELAY);
    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (!screen->dma_out && transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(spi->handle, &trans, portMAX_DELAY);
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
                spi_device_get_trans_result(spi->handle, &trans, portMAX_DELAY);
            }
            void *tmp = screen->pixels;
            screen->pixels = screen->dma_out;
            buf = screen->pixels;
            screen->dma_out = tmp;

            spidmawrite(spi, memsize, screen->dma_out);
        } else {
            spidmawrite(spi, memsize, buf);
        }

        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans;
        spi_device_get_trans_result(spi->handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(spi->handle);
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
    int spi_cs_gpio_atom_index = globalcontext_insert_atom(global, ATOM_STR("\xB", "spi_cs_gpio"));
    term spi_cs_gpio_atom = term_from_atom_index(spi_cs_gpio_atom_index);

    term cs_gpio_term = interop_proplist_get_value(opts, spi_cs_gpio_atom);
    if (cs_gpio_term == term_nil()) {
        return NULL;
    }

    int cs_gpio = term_to_int(cs_gpio_term);

    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, cs_gpio);
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

static void display_init(Context *ctx, int cs_gpio)
{
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

#if SD_ENABLE == true
    if (!sdcard_init()) {
#else
    {
#endif
        spi_bus_config_t buscfg;
        memset(&buscfg, 0, sizeof(spi_bus_config_t));
        buscfg.miso_io_num = MISO_IO_NUM;
        buscfg.mosi_io_num = MOSI_IO_NUM;
        buscfg.sclk_io_num = SCLK_IO_NUM;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;

        int ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);

        if (ret == ESP_OK) {
            fprintf(stderr, "initialized SPI\n");
        } else {
            fprintf(stderr, "spi_bus_initialize return code: %i\n", ret);
        }
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

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
    devcfg.clock_speed_hz = 1 * 1000 * 1000; //SPI_CLOCK_HZ;
    devcfg.mode = 1;
    devcfg.spics_io_num = cs_gpio;
    devcfg.queue_size = 32;
    devcfg.address_bits = ADDRESS_LEN_BITS;
    devcfg.flags = SPI_DEVICE_BIT_LSBFIRST | SPI_DEVICE_POSITIVE_CS;

    devcfg.cs_ena_pretrans = 4; // it should be at least 3us
    devcfg.cs_ena_posttrans = 2; // it should be at least 1us

    int ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi->handle);

    if (ret == ESP_OK) {
        fprintf(stderr, "initialized SPI device\n");
    } else {
        fprintf(stderr, "spi_bus_add_device return code: %i\n", ret);
    }

    gpio_set_direction(DC_IO_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(DC_IO_NUM, 1);

    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
}
