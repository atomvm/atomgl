/*
 * This file is part of AtomGL.
 *
 * Copyright 2024 Davide Bettio <davide@uninstall.it>
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

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <context.h>
#include <interop.h>

#include <i2c_driver.h>

#include "display_common.h"

#define TAG "SSD1306"

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define PAGE_HEIGHT 8
#define PAGES_NUM 8
#define CHAR_WIDTH 8

#define I2C_ADDRESS 0x3C

#define CTRL_BYTE_CMD_SINGLE 0x80
#define CTRL_BYTE_CMD_STREAM 0x00
#define CTRL_BYTE_DATA_STREAM 0x40

#define CMD_DISPLAY_INVERTED 0xA7
#define CMD_DISPLAY_ON 0xAF
#define CMD_SET_SEGMENT_REMAP 0xA1
#define CMD_SET_COM_SCAN_MODE 0xC8
#define CMD_SET_CHARGE_PUMP 0x8D

typedef enum
{
    DISPLAY_SSD1306,
    DISPLAY_SSD1315,
    DISPLAY_SH1106,
} display_type_t;

// TODO: let's change name, since also non SPI display are supported now
struct SPI
{
    term i2c_host;
    display_type_t type;
    Context *ctx;
};

static void do_update(Context *ctx, term display_list);

#include "font.c"
#include "display_items.h"
#include "draw_common.h"
#include "monochrome.h"
#include "message_helpers.h"

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

    int memsize = (DISPLAY_WIDTH * (PAGE_HEIGHT + 1)) / sizeof(uint8_t);
    uint8_t *buf = malloc(memsize);
    memset(buf, 0, memsize);

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(spi->i2c_host, &i2c_num, ctx->global) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        return;
    }

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(buf, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        uint8_t *out_buf = buf + (DISPLAY_WIDTH / 8);
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
            out_buf[i] |= ((buf[i / 8] >> (i % 8)) & 1) << (ypos % 8);
        }

        if ((ypos % PAGE_HEIGHT) == (PAGE_HEIGHT - 1)) {
            i2c_cmd_handle_t cmd;
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
            i2c_master_write_byte(cmd, 0xB0 | ypos / 8, true);

            if (spi->type == DISPLAY_SH1106 || spi->type == DISPLAY_SSD1315) {
                // SSD1315 and SH1106 require explicit column address reset
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, 0x00, true);
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, 0x10, true);
            }
            i2c_master_write_byte(cmd, CTRL_BYTE_DATA_STREAM, true);


            if (spi->type == DISPLAY_SH1106) {
                // add 2 empty pages on sh1106 since it can have up to 132 pixels
                // and 128 pixel screen starts at (2, 0)
                i2c_master_write_byte(cmd, 0, true);
                i2c_master_write_byte(cmd, 0, true);
            }

            for (uint8_t j = 0; j < DISPLAY_WIDTH; j++) {
                i2c_master_write_byte(cmd, out_buf[j], true);
            }

            // no need to send the last 2 page, the position will be set on next line again
            // if (spi->type == DISPLAY_SH1106) {
            //    i2c_master_write_byte(cmd, 0, true);
            //    i2c_master_write_byte(cmd, 0, true);
            // }

            i2c_master_stop(cmd);
            i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            memset(buf, 0, memsize);
        }
    }

    i2c_driver_release(spi->i2c_host, ctx->global);

    free(buf);
    destroy_items(items, len);
}

static void display_init(Context *ctx, term opts)
{
    GlobalContext *glb = ctx->global;

    term i2c_host
        = interop_kv_get_value_default(opts, ATOM_STR("\x8", "i2c_host"), term_invalid_term(), glb);
    if (i2c_host == term_invalid_term()) {
        ESP_LOGE(TAG, "Missing i2c_host config option.");
        return;
    }

    bool invert = interop_kv_get_value(opts, ATOM_STR("\x6", "invert"), glb) == TRUE_ATOM;

    display_messages_queue = xQueueCreate(32, sizeof(Message *));

    struct SPI *spi = malloc(sizeof(struct SPI));
    ctx->platform_data = spi;

    spi->ctx = ctx;
    spi->type = DISPLAY_SSD1306; // Default to SSD1306

    term compat_value_term = interop_kv_get_value_default(opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_value_term, &str_ok);

    if (!(str_ok && compat_string)) {
        ESP_LOGE(TAG, "No Compatible Device Found.");
        return;
    }

    if (!strcmp(compat_string, "sino-wealth,sh1106")) {
        spi->type = DISPLAY_SH1106;
    } else if (!strcmp(compat_string, "solomon-systech,ssd1315")) {
        spi->type = DISPLAY_SSD1315;
    }

    free(compat_string);

    int reset_gpio;
    if (!display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &reset_gpio, glb)) {
        ESP_LOGI(TAG, "Reset GPIO not configured.");
    } else {
        gpio_set_direction(reset_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(reset_gpio, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(reset_gpio, 1);
    }

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(i2c_host, &i2c_num, glb) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        return;
    }
    spi->i2c_host = i2c_host;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CTRL_BYTE_CMD_STREAM, true);

    if (spi->type == DISPLAY_SSD1315) {
        /*
         * Init sequence derived from u8g2 project (BSD-2-Clause).
         * Source: https://github.com/olikraus/u8g2
         *
         * These values are standard hardware initialization commands
         * defined by the Solomon Systech SSD1315 datasheet.
         */

        i2c_master_write_byte(cmd, 0xAE, true);  // Display OFF

        i2c_master_write_byte(cmd, 0xD5, true);  // Set Display Clock Divide Ratio / Oscillator Frequency
        i2c_master_write_byte(cmd, 0x80, true);  // 0x80 is standard/stable

        i2c_master_write_byte(cmd, 0xA8, true);  // Set Multiplex Ratio
        i2c_master_write_byte(cmd, 0x3F, true);  // 64 MUX

        i2c_master_write_byte(cmd, 0xD3, true);  // Set Display Offset
        i2c_master_write_byte(cmd, 0x00, true);  // No offset

        i2c_master_write_byte(cmd, 0x40, true);  // Set Display Start Line to 0

        i2c_master_write_byte(cmd, 0x8D, true);  // Set Charge Pump
        i2c_master_write_byte(cmd, 0x14, true);  // Enable Charge Pump

        i2c_master_write_byte(cmd, 0xA1, true);  // Set Segment Remap
        i2c_master_write_byte(cmd, 0xC8, true);  // Set COM Scan Mode

        i2c_master_write_byte(cmd, 0xDA, true);  // Set COM Pins Hardware Configuration
        i2c_master_write_byte(cmd, 0x12, true);  // Alternative COM pin config

        i2c_master_write_byte(cmd, 0x81, true);  // Set Contrast Control
        i2c_master_write_byte(cmd, 0xCF, true);  // Use High Contrast (0xCF) as per u8x8

        i2c_master_write_byte(cmd, 0xD9, true);  // Set Pre-charge Period
        i2c_master_write_byte(cmd, 0xF1, true);  // 0xF1 is required for stable 400kHz operation

        i2c_master_write_byte(cmd, 0xDB, true);  // Set VCOMH Deselect Level
        i2c_master_write_byte(cmd, 0x40, true);  // 0x40 (approx 0.77x VCC)

        i2c_master_write_byte(cmd, 0xA4, true);  // Resume to RAM content display
        i2c_master_write_byte(cmd, 0xA6, true);  // Normal Display (not inverted)

        i2c_master_write_byte(cmd, 0xAD, true);  // Internal IREF Setting
        i2c_master_write_byte(cmd, 0x10, true);  // Internal Iref
    } else {
        i2c_master_write_byte(cmd, CMD_SET_CHARGE_PUMP, true);
        i2c_master_write_byte(cmd, 0x14, true);

        i2c_master_write_byte(cmd, CMD_SET_SEGMENT_REMAP, true);
        i2c_master_write_byte(cmd, CMD_SET_COM_SCAN_MODE, true);
    }

    if (invert) {
        i2c_master_write_byte(cmd, CMD_DISPLAY_INVERTED, true);
    }

    i2c_master_write_byte(cmd, CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(i2c_num, cmd, 50 / portTICK_PERIOD_MS);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "ssd1306/ssd1315 OLED configuration failed. error: 0x%.2X", res);
    } else {
        xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
    }

    i2c_cmd_link_delete(cmd);
    i2c_driver_release(i2c_host, glb);
}

Context *ssd1306_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);

    return ctx;
}
