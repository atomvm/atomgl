/***************************************************************************
 *   Copyright 2020,2021 by Davide Bettio <davide@uninstall.it>            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA .        *
 ***************************************************************************/

#include <display_driver.h>

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

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

#include <../platforms/esp32/main/esp32_sys.h>

#include <trace.h>

#define ENABLE_ILI9342C CONFIG_AVM_ILI934X_ENABLE_ILI9342C
#define ENABLE_TFT_INVON CONFIG_AVM_ILI934X_ENABLE_TFT_INVON

#define BACKLIGHT_IO_NUM CONFIG_AVM_ILI934X_BACKLIGHT_IO_NUM

#define RESET_IO_NUM CONFIG_AVM_ILI934X_RESET_IO_NUM
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

#define ILI9341_SLPIN 0x10
#define ILI9341_SLPOUT 0x11
#define ILI9341_PTLON 0x12
#define ILI9341_NORON 0x13

#define ILI9341_INVOFF 0x20
#define ILI9341_INVON 0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON 0x29

#define ILI9341_PTLAR 0x30
#define ILI9341_VSCRDEF 0x33
#define ILI9341_MADCTL 0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT 0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR 0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1 0xC0
#define ILI9341_PWCTR2 0xC1
#define ILI9341_PWCTR3 0xC2
#define ILI9341_PWCTR4 0xC3
#define ILI9341_PWCTR5 0xC4
#define ILI9341_VMCTR1 0xC5
#define ILI9341_VMCTR2 0xC7

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1

#define TFT_SWRST 0x01
#define TFT_CASET 0x2A
#define TFT_PASET 0x2B
#define TFT_RAMWR 0x2C

#define TFT_MADCTL 0x36
#define TFT_MAD_MY 0x80
#define TFT_MAD_MX 0x40
#define TFT_MAD_MV 0x20
#define TFT_MAD_BGR 0x08

#define TFT_INVOFF 0x20
#define TFT_INVON 0x21

#include "font.c"

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

static void display_driver_consume_mailbox(Context *ctx);
static void display_init(Context *ctx, term opts);
#if ENABLE_ILI93442C == true
static void display_init42c(struct SPI *spi);
#else
static void display_init41(struct SPI *spi);
#endif

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

static bool spiwrite(struct SPI *spi_data, int data_len, uint32_t data)
{
    memset(&spi_data->transaction, 0, sizeof(spi_transaction_t));

    uint32_t tx_data = SPI_SWAP_DATA_TX(data, data_len);

    spi_data->transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    spi_data->transaction.length = data_len;
    spi_data->transaction.addr = 0;
    spi_data->transaction.tx_data[0] = tx_data;
    spi_data->transaction.tx_data[1] = (tx_data >> 8) & 0xFF;
    spi_data->transaction.tx_data[2] = (tx_data >> 16) & 0xFF;
    spi_data->transaction.tx_data[3] = (tx_data >> 24) & 0xFF;

    //TODO: int ret = spi_device_queue_trans(spi_data->handle, &spi_data->transaction, portMAX_DELAY);
    int ret = spi_device_polling_transmit(spi_data->handle, &spi_data->transaction);
    if (UNLIKELY(ret != ESP_OK)) {
        fprintf(stderr, "spiwrite: transmit error\n");
        return false;
    }

    return true;
}

static bool spidmawrite(struct SPI *spi_data, int data_len, const void *data)
{
    memset(&spi_data->transaction, 0, sizeof(spi_transaction_t));

    spi_data->transaction.flags = 0;
    spi_data->transaction.length = data_len * 8;
    spi_data->transaction.addr = 0;
    spi_data->transaction.tx_buffer = data;

    //TODO: int ret = spi_device_queue_trans(spi_data->handle, &spi_data->transaction, portMAX_DELAY);
    int ret = spi_device_polling_transmit(spi_data->handle, &spi_data->transaction);
    if (UNLIKELY(ret != ESP_OK)) {
        fprintf(stderr, "spidmawrite: transmit error\n");
        return false;
    }

    return true;
}

static inline void writedata(struct SPI *spi, uint32_t data)
{
    spi_device_acquire_bus(spi->handle, portMAX_DELAY);
    spiwrite(spi, 8, data);
    spi_device_release_bus(spi->handle);
}

static inline void writecommand(struct SPI *spi, uint8_t command)
{
    gpio_set_level(DC_IO_NUM, 0);
    writedata(spi, command);
    gpio_set_level(DC_IO_NUM, 1);
}

static inline void set_screen_paint_area(struct SPI *spi, int x, int y, int width, int height)
{
    writecommand(spi, TFT_CASET);
    spi_device_acquire_bus(spi->handle, portMAX_DELAY);
    spiwrite(spi, 32, (x << 16) | ((x + width) - 1));
    spi_device_release_bus(spi->handle);

    writecommand(spi, TFT_PASET);
    spi_device_acquire_bus(spi->handle, portMAX_DELAY);
    spiwrite(spi, 32, (y << 16) | ((y + height) - 1));
    spi_device_release_bus(spi->handle);
}

static void draw_rect(struct SPI *spi, int x, int y, int width, int height, uint8_t r, uint8_t g, uint8_t b)
{
    set_screen_paint_area(spi, x, y, width, height);

    writecommand(spi, TFT_RAMWR);

    int dest_size = width * height;
    int buf_pixels = (dest_size > 1024) ? 1024 : dest_size;

    uint16_t *tmpbuf = heap_caps_malloc(buf_pixels * sizeof(uint16_t), MALLOC_CAP_DMA);

    uint16_t fg_color = (((uint16_t)(r >> 3)) << 11) | (((uint16_t)(g >> 2)) << 5) | ((uint16_t) b >> 3);

    uint16_t color = SPI_SWAP_DATA_TX(fg_color, 16);

    for (int i = 0; i < buf_pixels; i++) {
        tmpbuf[i] = color;
    }

    spi_device_acquire_bus(spi->handle, portMAX_DELAY);

    int chunks = dest_size / 1024;
    for (int i = 0; i < chunks; i++) {
        spidmawrite(spi, 1024 * sizeof(uint16_t), tmpbuf);
    }
    int last_chunk_size = dest_size - chunks * 1024;
    if (last_chunk_size) {
        spidmawrite(spi, last_chunk_size * sizeof(uint16_t), tmpbuf);
    }

    spi_device_release_bus(spi->handle);
    free(tmpbuf);
}

static inline void clear_screen(struct SPI *spi, uint8_t r, uint8_t g, uint8_t b)
{
    draw_rect(spi, 0, 0, 320, 240, r, g, b);
}

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

static inline void rgba_to_rgb565(const uint8_t *data, int buf_pixel_size, uint16_t bg_color, uint16_t *outbuf)
{
    uint16_t swapped_bg_color = SPI_SWAP_DATA_TX(bg_color, 16);

    for (int i = 0; i < buf_pixel_size; i++) {
        if (data[3] == 255) {
            uint16_t color = (((uint16_t)(data[0] >> 3)) << 11) | (((uint16_t)(data[1] >> 2)) << 5) | ((uint16_t) data[2] >> 3);
            outbuf[i] = SPI_SWAP_DATA_TX(color, 16);

        } else if (data[3] == 0) {
            outbuf[i] = swapped_bg_color;

        } else {
            uint16_t fg_color = (((uint16_t)(data[0] >> 3)) << 11) | (((uint16_t)(data[1] >> 2)) << 5) | ((uint16_t) data[2] >> 3);

            uint16_t color = alpha_blend_rgb565(fg_color, bg_color, data[3]);
            outbuf[i] = SPI_SWAP_DATA_TX(color, 16);
        }
        data += 4;
    }
}

void draw_buffer(struct SPI *spi, int x, int y, int width, int height, const void *imgdata)
{
    const uint16_t *data = imgdata;

    set_screen_paint_area(spi, x, y, width, height);

    writecommand(spi, TFT_RAMWR);

    int dest_size = width * height;
    int buf_pixel_size = (dest_size > 1024) ? 1024 : dest_size;

    int chunks = dest_size / 1024;

    uint16_t *tmpbuf = heap_caps_malloc(buf_pixel_size * sizeof(uint16_t), MALLOC_CAP_DMA);

    spi_device_acquire_bus(spi->handle, portMAX_DELAY);
    for (int i = 0; i < chunks; i++) {
        const uint16_t *data_b = data + 1024 * i;
        for (int j = 0; j < 1024; j++) {
            tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
        }
        spidmawrite(spi, buf_pixel_size * sizeof(uint16_t), tmpbuf);
    }
    int last_chunk_size = dest_size - chunks * 1024;
    if (last_chunk_size) {
        const uint16_t *data_b = data + chunks * 1024;
        for (int j = 0; j < 1024; j++) {
            tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
        }
        spidmawrite(spi, last_chunk_size * sizeof(uint16_t), tmpbuf);
    }
    spi_device_release_bus(spi->handle);

    free(tmpbuf);
}

static void draw_image(struct SPI *spi, int x, int y, int width, int height, const void *imgdata, uint8_t r, uint8_t g, uint8_t b)
{
    const uint8_t *data = imgdata;

    set_screen_paint_area(spi, x, y, width, height);

    writecommand(spi, TFT_RAMWR);

    int dest_size = width * height;
    int buf_pixel_size = (dest_size > 1024) ? 1024 : dest_size;

    uint16_t *tmpbuf = heap_caps_malloc(buf_pixel_size * sizeof(uint16_t), MALLOC_CAP_DMA);

    uint16_t bg_color = (((uint16_t)(r >> 3)) << 11) | (((uint16_t)(g >> 2)) << 5) | ((uint16_t) b >> 3);

    int chunks = dest_size / 1024;

    spi_device_acquire_bus(spi->handle, portMAX_DELAY);
    for (int i = 0; i < chunks; i++) {
        rgba_to_rgb565(data + i * 1024 * sizeof(uint32_t), buf_pixel_size, bg_color, tmpbuf);
        spidmawrite(spi, buf_pixel_size * sizeof(uint16_t), tmpbuf);
    }
    int last_chunk_size = dest_size - chunks * 1024;
    if (last_chunk_size) {
        rgba_to_rgb565(data + chunks * 1024 * sizeof(uint32_t), last_chunk_size, bg_color, tmpbuf);
        spidmawrite(spi, last_chunk_size * sizeof(uint16_t), tmpbuf);
    }
    spi_device_release_bus(spi->handle);

    free(tmpbuf);
}

static void draw_text(struct SPI *spi, int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b)
{
    int len = strlen(text);

    uint16_t fg_color = (((uint16_t)(r >> 3)) << 11) | (((uint16_t)(g >> 2)) << 5) | ((uint16_t) b >> 3);

    for (int i = 0; i < len; i++) {
        unsigned const char *glyph = fontdata + ((unsigned char) text[i]) * 16;

        for (int j = 0; j < 16; j++) {
            unsigned char row = glyph[j];

            for (int k = 0; k < 8; k++) {
                if (row & (1 << (7 - k))) {
                    set_screen_paint_area(spi, x + i * 8 + k, y + j, 1, 1);
                    writecommand(spi, TFT_RAMWR);
                    spi_device_acquire_bus(spi->handle, portMAX_DELAY);
                    spiwrite(spi, 16, fg_color);
                    spi_device_release_bus(spi->handle);
                }
            }
        }
    }
}

static void process_message(Message *message, Context *ctx)
{
    term msg = message->message;

    term req = term_get_tuple_element(msg, 3);
    term cmd = term_get_tuple_element(req, 0);

    struct SPI *spi = ctx->platform_data;

    if (cmd == context_make_atom(ctx, "\xC"
                                      "clear_screen")) {
        int color = term_to_int(term_get_tuple_element(req, 1));

        clear_screen(spi, (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);

    } else if (cmd == context_make_atom(ctx, "\xA"
                                             "draw_image")) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        term img = term_get_tuple_element(req, 3);
        int color = term_to_int(term_get_tuple_element(req, 4));

        int width = term_to_int(term_get_tuple_element(img, 0));
        int height = term_to_int(term_get_tuple_element(img, 1));
        const char *data = term_binary_data(term_get_tuple_element(img, 2));

        draw_image(spi, x, y, width, height, data, (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);

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

    } else if (cmd == context_make_atom(ctx, "\x9"
                                             "draw_rect")) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        int color = term_to_int(term_get_tuple_element(req, 5));

        draw_rect(spi, x, y, width, height,
            (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);

    } else if (cmd == context_make_atom(ctx, "\x9"
                                             "draw_text")) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        term text_term = term_get_tuple_element(req, 3);
        int color = term_to_int(term_get_tuple_element(req, 4));

        int ok;
        char *text = interop_term_to_string(text_term, &ok);

        draw_text(spi, x, y, text, (color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);

        free(text);

    } else {
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
    }

    term pid = term_get_tuple_element(msg, 1);
    term ref = term_get_tuple_element(msg, 2);

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

void display_enqueue_message(Message *message)
{
    xQueueSend(display_messages_queue, &message, 1);
}

static void display_driver_consume_mailbox(Context *ctx)
{
    while (!list_is_empty(&ctx->mailbox)) {
        Message *message = mailbox_dequeue(ctx);
        xQueueSend(display_messages_queue, &message, 1);
    }
}

static void set_rotation(struct SPI *spi, int rotation)
{
    if (rotation == 1) {
        writecommand(spi, TFT_MADCTL);
        writedata(spi, TFT_MAD_BGR | TFT_MAD_MY | TFT_MAD_MV);
    }
}

void display_driver_init(Context *ctx, term opts)
{
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);
}

static void send_message(term pid, term message, GlobalContext *global)
{
    int local_process_id = term_to_local_process_id(pid);
    Context *target = globalcontext_get_process(global, local_process_id);
    if (LIKELY(target)) {
        mailbox_send(target, message);
    }
}

void display_callback(EventListener *listener)
{
    struct SPI *spi = listener->data;
    Context *ctx = spi->ctx;

    struct PendingReply pending;
    if (xQueueReceive(spi->replies_queue, &pending, 1)) {
        int ref_size = (sizeof(uint64_t) / sizeof(term)) + 1;
        if (UNLIKELY(memory_ensure_free(ctx, 3 + ref_size) != MEMORY_GC_OK)) {
            abort();
        }
        term return_tuple = term_alloc_tuple(2, ctx);

        term ref = term_from_ref_ticks(pending.pending_call_ref_ticks, ctx);

        term_put_tuple_element(return_tuple, 0, ref);
        term_put_tuple_element(return_tuple, 1, OK_ATOM);

        send_message(pending.pending_call_pid, return_tuple, ctx->global);
    }
}

static void display_init(Context *ctx, term opts)
{
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
    devcfg.clock_speed_hz = SPI_CLOCK_HZ;
    devcfg.mode = SPI_MODE;
    devcfg.spics_io_num = DISPLAY_CS_IO_NUM;
    devcfg.queue_size = 32;
    devcfg.address_bits = ADDRESS_LEN_BITS;

    int ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi->handle);

    if (ret == ESP_OK) {
        fprintf(stderr, "initialized SPI device\n");
    } else {
        fprintf(stderr, "spi_bus_add_device return code: %i\n", ret);
    }

    // Reset
    spi_device_acquire_bus(spi->handle, portMAX_DELAY);
    gpio_set_direction(RESET_IO_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_IO_NUM, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RESET_IO_NUM, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RESET_IO_NUM, 1);
    spi_device_release_bus(spi->handle);

    gpio_set_direction(BACKLIGHT_IO_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(BACKLIGHT_IO_NUM, 1);

    gpio_set_direction(DC_IO_NUM, GPIO_MODE_OUTPUT);

    writecommand(spi, TFT_SWRST);

    vTaskDelay(5 / portTICK_PERIOD_MS);

#if ENABLE_ILI93442C == true
    display_init42c(spi);
#else
    display_init41(spi);
#endif

    writecommand(spi, ILI9341_SLPOUT);

    vTaskDelay(120 / portTICK_PERIOD_MS);

    writecommand(spi, ILI9341_DISPON);

#if ENABLE_TFT_INVON == true
    writecommand(spi, TFT_INVON);
#endif

    set_rotation(spi, 1);

    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
}

#if ENABLE_ILI93442C == false
static void display_init41(struct SPI *spi)
{
    writecommand(spi, 0xEF);
    writedata(spi, 0x03);
    writedata(spi, 0x80);
    writedata(spi, 0x02);

    writecommand(spi, 0xCF);
    writedata(spi, 0x00);
    writedata(spi, 0xC1);
    writedata(spi, 0x30);

    writecommand(spi, 0xED);
    writedata(spi, 0x64);
    writedata(spi, 0x03);
    writedata(spi, 0x12);
    writedata(spi, 0x81);

    writecommand(spi, 0xE8);
    writedata(spi, 0x85);
    writedata(spi, 0x00);
    writedata(spi, 0x78);

    writecommand(spi, 0xCB);
    writedata(spi, 0x39);
    writedata(spi, 0x2C);
    writedata(spi, 0x00);
    writedata(spi, 0x34);
    writedata(spi, 0x02);

    writecommand(spi, 0xF7);
    writedata(spi, 0x20);

    writecommand(spi, 0xEA);
    writedata(spi, 0x00);
    writedata(spi, 0x00);

    writecommand(spi, ILI9341_PWCTR1);
    writedata(spi, 0x23);

    writecommand(spi, ILI9341_PWCTR2);
    writedata(spi, 0x10);

    writecommand(spi, ILI9341_VMCTR1);
    writedata(spi, 0x3E);
    writedata(spi, 0x28);

    writecommand(spi, ILI9341_VMCTR2);
    writedata(spi, 0x86);

    writecommand(spi, ILI9341_MADCTL);
    writedata(spi, 0x08);

    writecommand(spi, ILI9341_PIXFMT);
    writedata(spi, 0x55);

    writecommand(spi, ILI9341_FRMCTR1);
    writedata(spi, 0x00);
    writedata(spi, 0x13);

    writecommand(spi, ILI9341_DFUNCTR);
    writedata(spi, 0x0A);
    writedata(spi, 0xA2);
    writedata(spi, 0x27);

    writecommand(spi, 0xF2);
    writedata(spi, 0x00);

    writecommand(spi, ILI9341_GAMMASET);
    writedata(spi, 0x01);

    writecommand(spi, ILI9341_GMCTRP1);
    writedata(spi, 0x0F);
    writedata(spi, 0x31);
    writedata(spi, 0x2B);
    writedata(spi, 0x0C);
    writedata(spi, 0x0E);
    writedata(spi, 0x08);
    writedata(spi, 0x4E);
    writedata(spi, 0xF1);
    writedata(spi, 0x37);
    writedata(spi, 0x07);
    writedata(spi, 0x10);
    writedata(spi, 0x03);
    writedata(spi, 0x0E);
    writedata(spi, 0x09);
    writedata(spi, 0x00);

    writecommand(spi, ILI9341_GMCTRN1);
    writedata(spi, 0x00);
    writedata(spi, 0x0E);
    writedata(spi, 0x14);
    writedata(spi, 0x03);
    writedata(spi, 0x11);
    writedata(spi, 0x07);
    writedata(spi, 0x31);
    writedata(spi, 0xC1);
    writedata(spi, 0x48);
    writedata(spi, 0x08);
    writedata(spi, 0x0F);
    writedata(spi, 0x0C);
    writedata(spi, 0x31);
    writedata(spi, 0x36);
    writedata(spi, 0x0F);
}
#endif

#if ENABLE_ILI93442C == true
static void display_init42c(struct SPI *spi)
{
    writecommand(spi, 0xC8);
    writedata(spi, 0xFF);
    writedata(spi, 0x93);
    writedata(spi, 0x42);

    writecommand(spi, ILI9341_PWCTR1);
    writedata(spi, 0x12);
    writedata(spi, 0x12);

    writecommand(spi, ILI9341_PWCTR2);
    writedata(spi, 0x03);

    writecommand(spi, 0xB0);
    writedata(spi, 0xE0);

    writecommand(spi, 0xF6);
    writedata(spi, 0x00);
    writedata(spi, 0x01);
    writedata(spi, 0x01);

    writecommand(spi, ILI9341_MADCTL);
    writedata(spi, TFT_MAD_MY | TFT_MAD_MV);

    writecommand(spi, ILI9341_PIXFMT);
    writedata(spi, 0x55);

    writecommand(spi, ILI9341_DFUNCTR);
    writedata(spi, 0x08);
    writedata(spi, 0x82);
    writedata(spi, 0x27);

    writecommand(spi, ILI9341_GMCTRP1);
    writedata(spi, 0x00);
    writedata(spi, 0x0C);
    writedata(spi, 0x11);
    writedata(spi, 0x04);
    writedata(spi, 0x11);
    writedata(spi, 0x08);
    writedata(spi, 0x37);
    writedata(spi, 0x89);
    writedata(spi, 0x4C);
    writedata(spi, 0x06);
    writedata(spi, 0x0C);
    writedata(spi, 0x0A);
    writedata(spi, 0x2E);
    writedata(spi, 0x34);
    writedata(spi, 0x0F);

    writecommand(spi, ILI9341_GMCTRN1);
    writedata(spi, 0x00);
    writedata(spi, 0x0B);
    writedata(spi, 0x11);
    writedata(spi, 0x05);
    writedata(spi, 0x13);
    writedata(spi, 0x09);
    writedata(spi, 0x33);
    writedata(spi, 0x67);
    writedata(spi, 0x48);
    writedata(spi, 0x07);
    writedata(spi, 0x0E);
    writedata(spi, 0x0B);
    writedata(spi, 0x2E);
    writedata(spi, 0x33);
    writedata(spi, 0x0F);
}
#endif
