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
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <port.h>
#include <term.h>
#include <utils.h>

#include <i2c_driver.h>

#include "display_common.h"
#include "display_message.h"
#include "display_task.h"
#include "image_helpers.h"
#include "mono_draw.h"
#include "oled_commands.h"

#define TAG "oled_display"

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define PAGE_HEIGHT 8
#define PAGES_NUM 8

#define CTRL_BYTE_CMD_SINGLE 0x80
#define CTRL_BYTE_CMD_STREAM 0x00
#define CTRL_BYTE_DATA_STREAM 0x40

#define CMD_DISPLAY_INVERTED 0xA7
#define CMD_DISPLAY_ON 0xAF

struct OLEDDriver
{
    term i2c_host;
    const struct OLEDDesc *desc;
    Context *ctx;

    struct MonoScreen screen;

    struct DisplayTaskArgs display_args;
};

#define OLED_DRIVER_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct OLEDDriver, display_args)

#include "font_data.h"
#include "display_items.h"

static const struct {
    const char *compat;
    const struct OLEDDesc *desc;
} oled_compat_table[] = {
    { "solomon-systech,ssd1306", &oled_desc_ssd1306 },
    { "solomon-systech,ssd1315", &oled_desc_ssd1315 },
    { "sino-wealth,sh1106", &oled_desc_sh1106 },
};

static const struct OLEDDesc *oled_desc_for_compatible(const char *compat)
{
    for (size_t i = 0; i < sizeof(oled_compat_table) / sizeof(oled_compat_table[0]); i++) {
        if (!strcmp(compat, oled_compat_table[i].compat)) {
            return oled_compat_table[i].desc;
        }
    }
    return NULL;
}

static void do_update(Context *ctx, term display_list)
{
    BaseDisplayItem *items;
    size_t len;
    if (UNLIKELY(display_items_new_list(display_list, &items, &len, ctx) != DisplayItemsOk)) {
        return;
    }

    int screen_width = DISPLAY_WIDTH;
    int screen_height = DISPLAY_HEIGHT;
    struct OLEDDriver *driver = OLED_DRIVER_FROM_CTX(ctx);

    int memsize = (DISPLAY_WIDTH * (PAGE_HEIGHT + 1)) / sizeof(uint8_t);
    uint8_t *buf = malloc(memsize);
    if (UNLIKELY(!buf)) {
        fprintf(stderr, "do_update: failed to alloc buf\n");
        display_items_delete(items, len);
        return;
    }
    memset(buf, 0, memsize);

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(driver->i2c_host, &i2c_num, ctx->global) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        free(buf);
        display_items_delete(items, len);
        return;
    }

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = mono_draw_x(&driver->screen, buf, xpos, ypos, items, len);
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
            i2c_master_write_byte(cmd, (driver->desc->i2c_address << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
            i2c_master_write_byte(cmd, 0xB0 | ypos / 8, true);

            if (driver->desc->column_reset_per_page) {
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, 0x00, true);
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, 0x10, true);
            }
            i2c_master_write_byte(cmd, CTRL_BYTE_DATA_STREAM, true);

            // Pad the data stream with leading zero bytes for controllers
            // whose RAM is wider than the visible pixel area (SH1106 exposes
            // 128 of 132 columns starting at offset 2).
            for (uint8_t k = 0; k < driver->desc->scanline_prefix_pad_bytes; k++) {
                i2c_master_write_byte(cmd, 0, true);
            }

            for (uint8_t j = 0; j < DISPLAY_WIDTH; j++) {
                i2c_master_write_byte(cmd, out_buf[j], true);
            }

            i2c_master_stop(cmd);
            i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            memset(buf, 0, memsize);
        }
    }

    i2c_driver_release(driver->i2c_host, ctx->global);

    free(buf);
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

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {
        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);
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

    struct OLEDDriver *driver = malloc(sizeof(struct OLEDDriver));

    driver->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    driver->display_args.process_message_fn = process_message;
    driver->display_args.ctx = ctx;
    ctx->platform_data = &driver->display_args;

    driver->ctx = ctx;

    term compat_value_term = interop_kv_get_value_default(
        opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_value_term, &str_ok);
    const struct OLEDDesc *desc = NULL;
    if (str_ok && compat_string) {
        desc = oled_desc_for_compatible(compat_string);
    }
    if (!desc) {
        ESP_LOGE(TAG, "Failed init: unknown or missing compatible '%s'.",
            compat_string ? compat_string : "(null)");
        free(compat_string);
        return;
    }
    free(compat_string);

    driver->desc = desc;
    driver->screen.w = desc->native_width;
    driver->screen.h = desc->native_height;

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
    driver->i2c_host = i2c_host;

    oled_execute_init_seq(i2c_num, desc->i2c_address, desc->init_seq, desc->init_seq_len);

    // Driver-controlled finalization: optional invert, then display ON.
    // These depend on a runtime opt and are not panel data, so they
    // stay outside the per-variant init array.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (desc->i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CTRL_BYTE_CMD_STREAM, true);
    if (invert) {
        i2c_master_write_byte(cmd, CMD_DISPLAY_INVERTED, true);
    }
    i2c_master_write_byte(cmd, CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(i2c_num, cmd, 50 / portTICK_PERIOD_MS);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "%s configuration failed. error: 0x%.2X", desc->name, res);
    } else {
        xTaskCreate(display_task_process_messages, "display", 10000, &driver->display_args, 1, NULL);
    }

    i2c_cmd_link_delete(cmd);
    i2c_driver_release(i2c_host, glb);
}

Context *oled_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_init(ctx, opts);

    return ctx;
}
