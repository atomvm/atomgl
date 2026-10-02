/*
 * This file is part of AtomGL.
 *
 * Copyright 2022-2026 Davide Bettio <davide@uninstall.it>
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

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <stdbool.h>
#include <string.h>

#include <sys/time.h>

#include <defaultatoms.h>
#include <interop.h>
#include <mailbox.h>
#include <port.h>
#include <term.h>

#include <esp32_sys.h>

#include "display_items.h"
#include "display_message.h"
#include "display_task.h"
#include "display_common.h"
#include "epaper_color.h"
#include "epaper_commands.h"
#include "epaper_draw.h"
#include "epaper_screen.h"
#include "image_helpers.h"
#include "spi_dc_driver.h"
#include "spi_display.h"

#define REPORT_UNEXPECTED_MSGS 0
#define SELF_TEST 0

static const char *TAG = "epaper_display_driver";

static void clear_screen(Context *ctx, int color);

struct EpaperDriver
{
    struct SPIDCBus bus;

    int busy_gpio;
    int reset_gpio;

    const struct EPaperDesc *desc;
    struct EpaperScreen screen;

    Context *ctx;

    int count_to_refresh;
    uint64_t last_refresh;

    struct DisplayTaskArgs display_args;
};

#define EPAPER_DRIVER_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct EpaperDriver, display_args)

static const struct {
    const char *compat;
    const struct EPaperDesc *desc;
} epaper_compat_table[] = {
    { "waveshare,5in65-acep-7c", &epaper_desc_acep7c },
    { "good-display/gdep073e01", &epaper_desc_gdep073e01 },
};

static const struct EPaperDesc *epaper_desc_for_compatible(const char *compat)
{
    for (size_t i = 0; i < sizeof(epaper_compat_table) / sizeof(epaper_compat_table[0]); i++) {
        if (!strcmp(compat, epaper_compat_table[i].compat)) {
            return epaper_compat_table[i].desc;
        }
    }
    return NULL;
}

static void display_init_using_list(struct EpaperDriver *driver, term init_list);

static void display_reset(struct EpaperDriver *driver)
{
    gpio_set_level(driver->reset_gpio, 0);
    vTaskDelay(100);
    gpio_set_level(driver->reset_gpio, 1);
}

static void wait_busy_level(struct EpaperDriver *driver, int level)
{
    while (gpio_get_level(driver->busy_gpio) != level) {
        vTaskDelay(100);
    }
}

static void wait_some_time(Context *ctx)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL);
    uint64_t delta = now - driver->last_refresh;
    if (delta < 2000) {
        // Wait 2 seconds before allowing a new refresh; undocumented but
        // empirically required or the panel drops updates.
        vTaskDelay((2000 - delta) / portTICK_PERIOD_MS);
    }
}

static void update_last_refresh_ts(Context *ctx)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    driver->last_refresh = tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL);
}

static void maybe_refresh(Context *ctx)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);
    if (driver->desc->periodic_refresh_interval <= 0) {
        return;
    }

    driver->count_to_refresh--;
    if (driver->count_to_refresh <= 0) {
        // 7 is the panel's white entry on both current palettes.
        clear_screen(ctx, 7);
        update_last_refresh_ts(ctx);
        driver->count_to_refresh = driver->desc->periodic_refresh_interval;
    }
}

static void send_frame_preamble(struct EpaperDriver *driver)
{
    if (driver->desc->frame_preamble_seq != NULL) {
        epaper_execute_init_seq(&driver->bus, driver->busy_gpio,
            driver->desc->frame_preamble_seq,
            driver->desc->frame_preamble_seq_len, false);
    }
}

static void send_post_frame_refresh(struct EpaperDriver *driver)
{
    // PON
    spi_dc_write_command(&driver->bus, 0x04);
    wait_busy_level(driver, 1);

    // DRF (+ optional data byte)
    spi_dc_write_command(&driver->bus, 0x12);
    if (driver->desc->refresh_has_data) {
        spi_dc_write_data_n(&driver->bus, &driver->desc->refresh_data_byte, 1);
    }
    wait_busy_level(driver, 1);

    // POF
    spi_dc_write_command(&driver->bus, 0x02);
    wait_busy_level(driver, driver->desc->post_power_off_busy_level);
}

static void do_update(Context *ctx, term display_list)
{
    maybe_refresh(ctx);
    wait_some_time(ctx);

    BaseDisplayItem *items;
    size_t len;
    if (UNLIKELY(display_items_new_list(display_list, &items, &len, ctx) != DisplayItemsOk)) {
        return;
    }
    BaseDisplayItem **row = malloc(sizeof(BaseDisplayItem *) * len);
    if (UNLIKELY(len > 0 && !row)) {
        fprintf(stderr, "do_update: failed to alloc row\n");
        display_items_delete(items, len);
        return;
    }

    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);
    int screen_width = driver->screen.w;
    int screen_height = driver->screen.h;

    send_frame_preamble(driver);

    // DTM — data transfer to panel memory.
    spi_dc_write_command(&driver->bus, 0x10);

    uint8_t *buf = heap_caps_malloc(screen_width / 2, MALLOC_CAP_DMA);
    if (UNLIKELY(!buf)) {
        fprintf(stderr, "do_update: failed to alloc buf\n");
        free(row);
        display_items_delete(items, len);
        return;
    }
    memset(buf, 0x11, screen_width / 2);

    bool transaction_in_progress = false;

    spi_device_acquire_bus(driver->bus.spi_disp.handle, portMAX_DELAY);

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        size_t row_len = display_items_row(items, len, ypos, row);
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = epaper_draw_x(&driver->screen, buf, xpos, ypos, row, row_len);
            xpos += drawn_pixels;
        }

        spi_display_dma_write(&driver->bus.spi_disp, screen_width / 2, buf);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(driver->bus.spi_disp.handle);

    free(buf);

    send_post_frame_refresh(driver);

    free(row);
    display_items_delete(items, len);

    update_last_refresh_ts(ctx);
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

static void clear_screen(Context *ctx, int color)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);
    int screen_width = driver->screen.w;
    int screen_height = driver->screen.h;

    send_frame_preamble(driver);

    spi_dc_write_command(&driver->bus, 0x10);

    uint8_t *buf = heap_caps_malloc(screen_width / 2, MALLOC_CAP_DMA);
    if (UNLIKELY(!buf)) {
        fprintf(stderr, "clear_screen: failed to alloc buf\n");
        return;
    }

    bool transaction_in_progress = false;

    spi_device_acquire_bus(driver->bus.spi_disp.handle, portMAX_DELAY);

    for (int i = 0; i < screen_height; i++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        // memset inside the loop so every scanline carries fresh data,
        // avoiding artefacts if a prior scanline left stale bytes.
        memset(buf, color | (color << 4), screen_width / 2);
        spi_display_dma_write(&driver->bus.spi_disp, screen_width / 2, buf);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(driver->bus.spi_disp.handle);

    free(buf);

    send_post_frame_refresh(driver);
}

static void display_spi_init(Context *ctx, term opts)
{
    // Resolve compatible string -> per-panel descriptor.
    term compat_term = interop_kv_get_value_default(
        opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_term, &str_ok);
    const struct EPaperDesc *desc = NULL;
    if (str_ok && compat_string) {
        desc = epaper_desc_for_compatible(compat_string);
    }
    if (!desc) {
        ESP_LOGE(TAG, "Failed init: unknown or missing compatible '%s'.",
            compat_string ? compat_string : "(null)");
        free(compat_string);
        return;
    }
    free(compat_string);

    struct EpaperDriver *driver = malloc(sizeof(struct EpaperDriver));
    // TODO check here

    driver->desc = desc;
    driver->ctx = ctx;
    driver->screen.w = desc->native_width;
    driver->screen.h = desc->native_height;
    driver->screen.palette = desc->palette;
    driver->screen.palette_size = desc->palette_size;

    driver->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    driver->display_args.process_message_fn = process_message;
    driver->display_args.ctx = ctx;
    ctx->platform_data = &driver->display_args;

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.clock_speed_hz = desc->spi_clock_hz;
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&driver->bus.spi_disp, &spi_config);

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x4", "busy"), &driver->busy_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &driver->bus.dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &driver->reset_gpio, ctx->global);
    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display GPIOs.");
        return;
    }

    gpio_set_direction(driver->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(driver->reset_gpio, 1);
    gpio_set_direction(driver->bus.dc_gpio, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(driver->bus.dc_gpio, GPIO_PULLUP_ENABLE);
    gpio_set_direction(driver->busy_gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(driver->busy_gpio, GPIO_PULLUP_ENABLE);
    gpio_set_level(driver->bus.dc_gpio, 0);

    display_reset(driver);

    wait_busy_level(driver, 1);

    // Init sequence: init_list opt overrides the descriptor default.
    term init_list = interop_kv_get_value_default(
        opts, ATOM_STR("\x9", "init_list"), term_nil(), ctx->global);
    if (init_list != term_nil()) {
        display_init_using_list(driver, init_list);
    } else {
        epaper_execute_init_seq(&driver->bus, driver->busy_gpio,
            desc->init_seq, desc->init_seq_len,
            desc->init_wait_busy_between_cmds);
    }

    update_last_refresh_ts(ctx);
    driver->count_to_refresh = 0;

#if SELF_TEST
    for (int i = 0; i < 8; i++) {
        fprintf(stderr, "color: %i\n", i);
        clear_screen(ctx, i);
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
    clear_screen(ctx, 1);

    while (1)
        ;
#else
    xTaskCreate(display_task_process_messages, "display", 10000, &driver->display_args, 1, NULL);
#endif
}

Context *epaper_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_spi_init(ctx, opts);
    return ctx;
}

// Erlang-side init override: accepts a list of
//   {CmdByte :: 0..255, Binary :: binary()}
//   {sleep_ms, Ms :: 0..255}
//   {wait_busy_level, Level :: 0 | 1}
// tuples and applies them in order.  Mirrors dcs_lcd's display_init_using_list
// with an added wait_busy_level clause, since e-paper controllers typically
// require BUSY-pin polling between commands that DCS LCDs do not.
static void display_init_using_list(struct EpaperDriver *driver, term init_list)
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
                spi_dc_write_cmd_data(&driver->bus, cmd, data, term_binary_size(data_term));
            } else if ((cmd_term == context_make_atom(driver->ctx, ATOM_STR("\x8", "sleep_ms")))
                && term_is_integer(data_term)) {
                vTaskDelay(term_to_int(data_term) / portTICK_PERIOD_MS);
            } else if ((cmd_term == context_make_atom(driver->ctx, ATOM_STR("\xF", "wait_busy_level")))
                && term_is_integer(data_term)) {
                wait_busy_level(driver, term_to_int(data_term));
            } else {
                break;
            }
        } else {
            break;
        }

        t = term_get_list_tail(t);
    }
    if (t != term_nil()) {
        fprintf(stderr, "Invalid init_list!\n");
    }
}
