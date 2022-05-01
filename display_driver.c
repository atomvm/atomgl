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

#define CHAR_WIDTH 8

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

enum primitive
{
    Invalid = 0,
    Image,
    Rect,
    Text
};

struct TextData
{
    uint32_t fgcolor;
    const char *text;
};

struct ImageData
{
    const char *pix;
};

struct BaseDisplayItem
{
    enum primitive primitive;
    int x;
    int y;
    int width;
    int height;
    uint32_t brcolor; /* bounding rect color */
    union
    {
        struct ImageData image_data;
        struct TextData text_data;
    } data;
};

typedef struct BaseDisplayItem BaseDisplayItem;

// This struct is just for compatibility reasons with the SDL display driver
// so it is possible to easily copy & paste code from there.
struct Screen
{
    int w;
    int h;
    uint16_t *pixels;
    uint16_t *pixels_out;
};

struct Screen *screen;

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

    int ret = spi_device_queue_trans(spi_data->handle, &spi_data->transaction, portMAX_DELAY);
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

static int draw_image_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor;
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
    int line_len = screen->w;

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

static void init_item(BaseDisplayItem *item, term req, Context *ctx)
{
    term cmd = term_get_tuple_element(req, 0);

    if (cmd == context_make_atom(ctx, "\x5"
                                      "image")) {
        item->primitive = Image;
        item->x = term_to_int(term_get_tuple_element(req, 1));
        item->y = term_to_int(term_get_tuple_element(req, 2));

        term bgcolor = term_get_tuple_element(req, 3);
        if (bgcolor == context_make_atom(ctx, "\xB"
                                              "transparent")) {
            item->brcolor = 0;
        } else {
            item->brcolor = ((uint32_t) term_to_int(bgcolor)) << 8 | 0xFF;
        }

        term img = term_get_tuple_element(req, 4);

        term format = term_get_tuple_element(img, 0);
        if (format != context_make_atom(ctx, "\x8"
                                             "rgba8888")) {
            fprintf(stderr, "unsupported image format: ");
            term_display(stderr, format, ctx);
            fprintf(stderr, "\n");
            return;
        }
        item->width = term_to_int(term_get_tuple_element(img, 1));
        item->height = term_to_int(term_get_tuple_element(img, 2));
        item->data.image_data.pix = term_binary_data(term_get_tuple_element(img, 3));

    } else if (cmd == context_make_atom(ctx, "\x4"
                                             "rect")) {
        item->primitive = Rect;
        item->x = term_to_int(term_get_tuple_element(req, 1));
        item->y = term_to_int(term_get_tuple_element(req, 2));
        item->width = term_to_int(term_get_tuple_element(req, 3));
        item->height = term_to_int(term_get_tuple_element(req, 4));
        item->brcolor = term_to_int(term_get_tuple_element(req, 5)) << 8 | 0xFF;

    } else if (cmd == context_make_atom(ctx, "\x4"
                                             "text")) {
        item->primitive = Text;
        item->x = term_to_int(term_get_tuple_element(req, 1));
        item->y = term_to_int(term_get_tuple_element(req, 2));

        item->data.text_data.fgcolor = term_to_int(term_get_tuple_element(req, 4)) << 8 | 0xFF;
        term bgcolor = term_get_tuple_element(req, 5);
        if (bgcolor == context_make_atom(ctx, "\xB"
                                              "transparent")) {
            item->brcolor = 0;
        } else {
            item->brcolor = ((uint32_t) term_to_int(bgcolor)) << 8 | 0xFF;
        }

        term font = term_get_tuple_element(req, 3);
        if (font != context_make_atom(ctx, "\xB"
                                           "default16px")) {
            fprintf(stderr, "unsupported font: ");
            term_display(stderr, font, ctx);
            fprintf(stderr, "\n");
            return;
        }

        term text_term = term_get_tuple_element(req, 6);
        int ok;
        item->data.text_data.text = interop_term_to_string(text_term, &ok);
        if (!ok) {
            fprintf(stderr, "invalid text.\n");
            return;
        }

        item->height = 16;
        item->width = strlen(item->data.text_data.text) * 8;

    } else {
        fprintf(stderr, "unexpected display list command: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");

        item->primitive = Invalid;
        item->x = -1;
        item->y = -1;
        item->width = 1;
        item->height = 1;
    }
}

static void destroy_items(BaseDisplayItem *items, int items_count)
{
    for (int i = 0; i < items_count; i++) {
        BaseDisplayItem *item = &items[i];

        switch (item->primitive) {
            case Image:
                break;

            case Rect:
                break;

            case Text:
                free((char *) item->data.text_data.text);
                break;

            default: {
                break;
            }
        }
    }

    free(items);
}

static void do_update(Context *ctx, term display_list)
{
    int proper;
    int len = term_list_length(display_list, &proper);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);

    term t = display_list;
    for (int i = len - 1; i >= 0; i--) {
        init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = screen->w;
    int screen_height = screen->h;
    struct SPI *spi = ctx->platform_data;

    set_screen_paint_area(spi, 0, 0, screen_width, screen_height);
    writecommand(spi, TFT_RAMWR);
    spi_device_acquire_bus(spi->handle, portMAX_DELAY);

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
            spi_device_get_trans_result(spi->handle, &trans, portMAX_DELAY);
        }

        //NEW CODE
        void *tmp = screen->pixels;
        screen->pixels = screen->pixels_out;
        screen->pixels_out = tmp;
        spidmawrite(spi, screen_width * sizeof(uint16_t), screen->pixels_out);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans;
        spi_device_get_trans_result(spi->handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(spi->handle);

    destroy_items(items, len);
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

    } else {
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
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

Context *display_create_port(GlobalContext *global, term opts)
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

void display_callback(EventListener *listener)
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
    screen = malloc(sizeof(struct Screen));
    // FIXME: hardcoded width and height
    screen->w = 320;
    screen->h = 240;
    screen->pixels = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);
    screen->pixels_out = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);

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

    set_rotation(spi, 0);

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
