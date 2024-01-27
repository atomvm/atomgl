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
#include "spi_display.h"

#define CHAR_WIDTH 8

#define DISPLAY_WIDTH 400

#define CHECK_OVERFLOW 1
#define REPORT_UNEXPECTED_MSGS 0

#include "font.c"

struct SPI
{
    struct SPIDisplay spi_disp;
    Context *ctx;
};

#include "display_items.h"
#include "draw_common.h"
#include "monochrome.h"

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

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx);
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

static void send_message(term pid, term message, GlobalContext *global);

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
    globalcontext_send_message(global, local_process_id, message);
}

static void display_init(Context *ctx, term opts)
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

    display_messages_queue = xQueueCreate(32, sizeof(Message *));

    GlobalContext *glb = ctx->global;

    struct SPI *spi = malloc(sizeof(struct SPI));
    ctx->platform_data = spi;

    spi->ctx = ctx;

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

    int en_gpio;
    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x2", "en"), &en_gpio, glb);

    if (ok) {
        gpio_set_direction(en_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(en_gpio, 1);
    }

    xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
}
