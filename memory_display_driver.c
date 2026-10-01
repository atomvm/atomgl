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
#include <port.h>
#include <sys.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>

#include <trace.h>

#include <math.h>

#include "display_common.h"
#include "display_task.h"
#include "mono_draw.h"
#include "spi_display.h"

#define DISPLAY_WIDTH 400
#define DISPLAY_HEIGHT 240

#define REPORT_UNEXPECTED_MSGS 0

#include "font_data.h"

struct MemoryLCDDriver
{
    struct SPIDisplay spi_disp;
    Context *ctx;

    struct MonoScreen screen;

    uint8_t *pixels;
    uint8_t *dma_out;
    int vcom;

    struct DisplayTaskArgs display_args;
};

#define MEMORY_LCD_DRIVER_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct MemoryLCDDriver, display_args)

#include "display_items.h"
#include "display_message.h"
#include "image_helpers.h"

static void display_init(Context *ctx, term opts);

static inline int next_vcom(struct MemoryLCDDriver *driver)
{
    int current_vcom = driver->vcom;
    driver->vcom = current_vcom ? 0 : 0x2;
    return current_vcom;
}

static void do_update(Context *ctx, term display_list)
{
    BaseDisplayItem *items;
    size_t len;
    if (UNLIKELY(display_items_new_list(display_list, &items, &len, ctx) != DisplayItemsOk)) {
        return;
    }

    struct MemoryLCDDriver *driver = MEMORY_LCD_DRIVER_FROM_CTX(ctx);
    int screen_width = driver->screen.w;
    int screen_height = driver->screen.h;

    int memsize = 2 + 400 / 8 + 2;
    uint8_t *buf = driver->pixels;

    spi_device_acquire_bus(driver->spi_disp.handle, portMAX_DELAY);
    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (!driver->dma_out && transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(driver->spi_disp.handle, &trans, portMAX_DELAY);
        }

        memset(buf + 2, 0xFF, DISPLAY_WIDTH / 8);

        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = mono_draw_x(&driver->screen, buf + 2, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        buf[0] = 0x1 | next_vcom(driver);
        buf[1] = ypos + 1;
        buf[2 + DISPLAY_WIDTH / 8] = 0;
        buf[2 + DISPLAY_WIDTH / 8 + 1] = 0;

        if (driver->dma_out) {
            if (transaction_in_progress) {
                spi_transaction_t *trans = NULL;
                spi_device_get_trans_result(driver->spi_disp.handle, &trans, portMAX_DELAY);
            }
            void *tmp = driver->pixels;
            driver->pixels = driver->dma_out;
            buf = driver->pixels;
            driver->dma_out = tmp;

            spi_display_dma_write(&driver->spi_disp, memsize, driver->dma_out);
        } else {
            spi_display_dma_write(&driver->spi_disp, memsize, buf);
        }

        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans;
        spi_device_get_trans_result(driver->spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(driver->spi_disp.handle);
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
#if REPORT_UNEXPECTED_MSGS
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
#endif
    }

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);

    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

Context *memory_lcd_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
}

static void display_init(Context *ctx, term opts)
{
    GlobalContext *glb = ctx->global;

    term width_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x5", "width"), term_from_int(DISPLAY_WIDTH), glb);
    term height_term = interop_kv_get_value_default(
        opts, ATOM_STR("\x6", "height"), term_from_int(DISPLAY_HEIGHT), glb);
    int width = term_to_int(width_term);
    int height = term_to_int(height_term);

    int memsize = 2 + 400 / 8 + 2;

    struct MemoryLCDDriver *driver = malloc(sizeof(struct MemoryLCDDriver));

    driver->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    driver->display_args.process_message_fn = process_message;
    driver->display_args.ctx = ctx;
    ctx->platform_data = &driver->display_args;

    driver->ctx = ctx;
    driver->screen.w = width;
    driver->screen.h = height;
    driver->vcom = 0;

    driver->pixels = heap_caps_malloc(memsize, MALLOC_CAP_DMA);
    if (UNLIKELY(!driver->pixels)) {
        fprintf(stderr, "failed to allocate buf!\n");
        abort();
    }

    driver->dma_out = heap_caps_malloc(memsize, MALLOC_CAP_DMA);
    if (UNLIKELY(!driver->dma_out)) {
        fprintf(stderr, "failed to allocate buf!\n");
        abort();
    }

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.mode = 0;
    spi_config.clock_speed_hz = 1000000;
    spi_config.cs_active_high = true;
    spi_config.bit_lsb_first = true;
    spi_config.cs_ena_pretrans = 4; // it should be at least 3us
    spi_config.cs_ena_posttrans = 2; // it should be at least 1us
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&driver->spi_disp, &spi_config);

    int en_gpio;
    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x2", "en"), &en_gpio, glb);

    if (ok) {
        gpio_set_direction(en_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(en_gpio, 1);
    }

    xTaskCreate(display_task_process_messages, "display", 10000, &driver->display_args, 1, NULL);
}
