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
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "epaper_program.h"
#include "epaper_screen.h"
#include "image_helpers.h"
#include "mono_draw.h"
#include "spi_dc_driver.h"
#include "spi_display.h"

#define REPORT_UNEXPECTED_MSGS 0
#define SELF_TEST 0

static const char *TAG = "epaper_display_driver";

#define SSD1680_LUT_PAYLOAD_LEN 153
#define SSD1680_LUT_WITH_REGS_LEN 159
#define UC8151_LUT_PAYLOAD_LEN 30
#define UC8276_LUT_PAYLOAD_LEN 227
#define UC8276_LUT_WITH_REGS_LEN 233
#define EPAPER_TERM_PROGRAM_COUNT 5

struct EPaperState
{
    bool prev_valid;
    int fast_refresh_count;
    enum EPaperRefreshMode last_refresh_mode;
    uint64_t last_refresh_ms;
    bool needs_reseed;
    bool last_op_ok;
    bool asleep;
    const struct EPaperSleepMode *asleep_mode;
    bool after_wake_pending;
    bool after_wake_can_reseed_prev;
    enum EPaperAfterWakeRefreshPolicy after_wake_refresh;
};

static void clear_screen_internal(Context *ctx, int color,
    bool clear_after_wake_pending);

struct EpaperDriver
{
    struct SPIDCBus bus;

    int busy_gpio;
    int reset_gpio;

    const struct EPaperDesc *desc;
    struct EPaperDesc term_desc;
    char *term_desc_name;
    uint8_t *term_init_seq_bytes;
    uint8_t *term_frame_preamble_seq_bytes;
    uint8_t *term_program_bytes[EPAPER_TERM_PROGRAM_COUNT];
    uint8_t *term_sleep_enter_bytes[EPAPER_MAX_SLEEP_MODES];
    uint8_t *term_sleep_wake_bytes[EPAPER_MAX_SLEEP_MODES];
    uint8_t *term_lut_bytes[EPAPER_MAX_LUT_SLOTS];

    struct EpaperScreen screen;
    struct MonoScreen mono_screen;

    uint8_t *prev_frame;
    size_t prev_frame_len;

    struct EPaperState state;

    Context *ctx;

    int count_to_refresh;

    struct DisplayTaskArgs display_args;
};

#define EPAPER_DRIVER_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct EpaperDriver, display_args)

static const struct EPaperDesc epaper_desc_term_default = {
    .name = "Erlang e-paper descriptor",
    .native_width = 128,
    .native_height = 296,
    .view_width = 128,
    .view_height = 296,
    .rotation = 0,
    .spi_clock_hz = 4000000,
    .palette = NULL,
    .palette_size = 0,
    .controller = EPAPER_CONTROLLER_SSD16XX,
    .layout = {
        .byte_order = EPAPER_BYTE_ORDER_ROW_MAJOR,
        .bit_order = EPAPER_BIT_ORDER_MSB_LEFT,
        .polarity = EPAPER_POLARITY_WHITE_IS_1
    },
    .use_gpio_pullups = false,
    .busy_idle_level = 0,
    .refresh_mode_mask = EPAPER_REFRESH_MODE_FULL,
    .descriptor_version = 3,
};

static bool epaper_desc_requires_program(const struct EPaperDesc *desc)
{
    return desc->controller != EPAPER_CONTROLLER_ACEP7;
}

static void display_init_using_list(struct EpaperDriver *driver, term init_list);
static bool epaper_parse_descriptor_override(struct EpaperDriver *driver,
    term opts, Context *ctx);
static void epaper_free_init_failed_driver(struct EpaperDriver *driver,
    bool spi_device_added);
static bool epaper_parse_refresh_mode(term val, Context *ctx, enum EPaperRefreshMode *out);
static bool epaper_parse_palette(term val, Context *ctx,
    const uint8_t (**palette)[3], int *palette_size);

static void display_reset(struct EpaperDriver *driver)
{
    gpio_set_level(driver->reset_gpio, 0);
    vTaskDelay(100);
    gpio_set_level(driver->reset_gpio, 1);
}

static void epaper_delay_ms(uint32_t delay_ms)
{
    if (delay_ms == 0) {
        return;
    }
    if (delay_ms < portTICK_PERIOD_MS) {
        esp_rom_delay_us(delay_ms * 1000);
        return;
    }
    vTaskDelay((delay_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}

static void wait_busy_level(struct EpaperDriver *driver, int level)
{
    while (gpio_get_level(driver->busy_gpio) != level) {
        vTaskDelay(100);
    }
}

// Wait for BUSY pin to go to target level.  Returns true on
// success, false on timeout.  On timeout, invalidates the cached
// partial-refresh previous frame so the next partial update re-seeds
// via a full base refresh.
static bool wait_busy_timeout(struct EpaperDriver *driver, int level,
    uint16_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while (gpio_get_level(driver->busy_gpio) != level) {
        if ((xTaskGetTickCount() - start) > timeout) {
            if (driver->state.prev_valid) {
                ESP_LOGW(TAG,
                    "BUSY wait for level %d timed out for %s; invalidating partial prev frame.",
                    level, driver->desc->name);
            } else {
                ESP_LOGW(TAG, "BUSY wait for level %d timed out for %s.",
                    level, driver->desc->name);
            }
            driver->state.prev_valid = false;
            if (driver->desc->reseed_on_timeout) {
                driver->state.needs_reseed = true;
            }
            return false;
        }
        epaper_delay_ms(10);
    }
    return true;
}

static bool wait_busy_low_timeout(struct EpaperDriver *driver)
{
    int timeout = (driver->desc->timeout_ms > 0) ? driver->desc->timeout_ms : 5000;
    return wait_busy_timeout(driver, 0, timeout);
}

static bool epaper_write_lut_bytes(struct EpaperDriver *driver,
    const uint8_t *lut, size_t lut_len)
{
    if (lut == NULL || lut_len == 0) {
        return false;
    }

    if (driver->desc->controller == EPAPER_CONTROLLER_UC8151) {
        if (lut_len < UC8151_LUT_PAYLOAD_LEN) {
            ESP_LOGE(TAG, "UC8151 LUT is too short: %u bytes.", (unsigned) lut_len);
            return false;
        }
        spi_dc_write_cmd_data(&driver->bus, 0x32, lut, UC8151_LUT_PAYLOAD_LEN);
        return true;
    }

    if (driver->desc->controller == EPAPER_CONTROLLER_UC8276) {
        if (lut_len < UC8276_LUT_PAYLOAD_LEN) {
            ESP_LOGE(TAG, "UC8276 LUT is too short: %u bytes.", (unsigned) lut_len);
            return false;
        }
        spi_dc_write_cmd_data(&driver->bus, 0x32, lut, UC8276_LUT_PAYLOAD_LEN);

        if (lut_len < UC8276_LUT_WITH_REGS_LEN) {
            return true;
        }

        spi_dc_write_command(&driver->bus, 0x3F);
        spi_dc_write_data(&driver->bus, lut[227]);
        spi_dc_write_command(&driver->bus, 0x03);
        spi_dc_write_data(&driver->bus, lut[228]);
        spi_dc_write_command(&driver->bus, 0x04);
        spi_dc_write_data(&driver->bus, lut[229]);
        spi_dc_write_data(&driver->bus, lut[230]);
        spi_dc_write_data(&driver->bus, lut[231]);
        spi_dc_write_command(&driver->bus, 0x2C);
        spi_dc_write_data(&driver->bus, lut[232]);
        return true;
    }

    if (driver->desc->controller != EPAPER_CONTROLLER_SSD16XX) {
        ESP_LOGE(TAG, "Controller %d does not support LUT insertion.",
            driver->desc->controller);
        return false;
    }

    if (lut_len < SSD1680_LUT_PAYLOAD_LEN) {
        ESP_LOGE(TAG, "SSD1680 LUT is too short: %u bytes.", (unsigned) lut_len);
        return false;
    }

    spi_dc_write_cmd_data(&driver->bus, 0x32, lut, SSD1680_LUT_PAYLOAD_LEN);
    wait_busy_low_timeout(driver);

    if (lut_len < SSD1680_LUT_WITH_REGS_LEN) {
        return true;
    }

    spi_dc_write_command(&driver->bus, 0x3F); // WRITE_EOPQ
    spi_dc_write_data(&driver->bus, lut[153]);
    spi_dc_write_command(&driver->bus, 0x03); // GATE_VOLTAGE
    spi_dc_write_data(&driver->bus, lut[154]);
    spi_dc_write_command(&driver->bus, 0x04); // SOURCE_VOLTAGE
    spi_dc_write_data(&driver->bus, lut[155]);
    spi_dc_write_data(&driver->bus, lut[156]);
    spi_dc_write_data(&driver->bus, lut[157]);
    spi_dc_write_command(&driver->bus, 0x2C); // WRITE_VCOM
    spi_dc_write_data(&driver->bus, lut[158]);
    return true;
}

static void wait_some_time(Context *ctx)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL);
    uint64_t delta = now - driver->state.last_refresh_ms;
    if (delta < 2000) {
        // Wait 2 seconds before allowing a new refresh; undocumented but
        // empirically required or the panel drops updates.
        epaper_delay_ms(2000 - delta);
    }
}

static void update_last_refresh_ts(Context *ctx)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    driver->state.last_refresh_ms = tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL);
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
        clear_screen_internal(ctx, 7, false);
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

static int view_mono_line_bytes(struct EpaperDriver *driver)
{
    return (driver->mono_screen.w + 7) / 8;
}

static int native_row_bytes(struct EpaperDriver *driver)
{
    return (driver->desc->native_width + 7) / 8;
}

static int native_column_bytes(struct EpaperDriver *driver)
{
    return (driver->desc->native_height + 7) / 8;
}

static int native_frame_chunk_bytes(struct EpaperDriver *driver)
{
    if (driver->desc->layout.byte_order == EPAPER_BYTE_ORDER_COLUMN_MAJOR) {
        return native_column_bytes(driver);
    }
    return native_row_bytes(driver);
}

static int native_frame_chunk_count(struct EpaperDriver *driver)
{
    if (driver->desc->layout.byte_order == EPAPER_BYTE_ORDER_COLUMN_MAJOR) {
        return driver->desc->native_width;
    }
    return driver->desc->native_height;
}

static size_t mono_frame_bytes(struct EpaperDriver *driver)
{
    if (driver->desc->layout.byte_order == EPAPER_BYTE_ORDER_COLUMN_MAJOR) {
        return (size_t) driver->desc->native_width * native_column_bytes(driver);
    }
    return (size_t) native_row_bytes(driver) * driver->desc->native_height;
}

static bool epaper_view_to_native(const struct EPaperDesc *desc,
    int vx, int vy, int *nx, int *ny)
{
    switch (desc->rotation) {
        case 0:
            *nx = vx;
            *ny = vy;
            break;
        case 90:
            *nx = vy;
            *ny = desc->native_height - 1 - vx;
            break;
        case 180:
            *nx = desc->native_width - 1 - vx;
            *ny = desc->native_height - 1 - vy;
            break;
        case 270:
            *nx = desc->native_width - 1 - vy;
            *ny = vx;
            break;
        default:
            return false;
    }

    return *nx >= 0 && *nx < desc->native_width
        && *ny >= 0 && *ny < desc->native_height;
}

static void render_mono_line_raw(struct EpaperDriver *driver,
    uint8_t *buf, int ypos, BaseDisplayItem *items, int items_len)
{
    int plane_bytes = view_mono_line_bytes(driver);
    memset(buf, 0xFF, plane_bytes);

    int xpos = 0;
    while (xpos < driver->mono_screen.w) {
        int drawn_pixels = mono_draw_x(&driver->mono_screen, buf, xpos, ypos, items, items_len);
        xpos += drawn_pixels;
    }
}

static bool ensure_prev_frame(struct EpaperDriver *driver)
{
    size_t frame_len = mono_frame_bytes(driver);
    if (driver->prev_frame != NULL
        && driver->prev_frame_len == frame_len) {
        return true;
    }

    free(driver->prev_frame);
    driver->prev_frame = heap_caps_malloc(frame_len, MALLOC_CAP_DMA);
    driver->prev_frame_len = frame_len;
    driver->state.prev_valid = false;

    if (UNLIKELY(!driver->prev_frame)) {
        driver->prev_frame_len = 0;
        ESP_LOGE(TAG, "Failed to allocate e-paper previous-frame buffer.");
        return false;
    }

    return true;
}

static void write_mono_frame(struct EpaperDriver *driver,
    const uint8_t *frame)
{
    bool transaction_in_progress = false;
    int chunk_bytes = native_frame_chunk_bytes(driver);
    int chunk_count = native_frame_chunk_count(driver);

    spi_device_acquire_bus(driver->bus.spi_disp.handle, portMAX_DELAY);

    for (int chunk = 0; chunk < chunk_count; chunk++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        spi_display_dma_write(&driver->bus.spi_disp, chunk_bytes,
            frame + ((size_t) chunk * chunk_bytes));
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(driver->bus.spi_disp.handle);
}

static void set_native_frame_bit(struct EpaperDriver *driver, uint8_t *frame,
    int nx, int ny, bool bit_value)
{
    size_t byte_index;
    int bit_pos;

    if (driver->desc->layout.byte_order == EPAPER_BYTE_ORDER_COLUMN_MAJOR) {
        byte_index = (size_t) nx * native_column_bytes(driver) + (ny / 8);
        bit_pos = ny % 8;
    } else {
        byte_index = (size_t) ny * native_row_bytes(driver) + (nx / 8);
        bit_pos = nx % 8;
    }

    if (driver->desc->layout.bit_order == EPAPER_BIT_ORDER_MSB_LEFT) {
        bit_pos = 7 - bit_pos;
    }

    uint8_t mask = (uint8_t) (1U << bit_pos);
    if (bit_value) {
        frame[byte_index] |= mask;
    } else {
        frame[byte_index] &= (uint8_t) ~mask;
    }
}

static void set_native_mono_pixel(struct EpaperDriver *driver, uint8_t *frame,
    int nx, int ny, bool white)
{
    bool bit_value = white;
    if (driver->desc->layout.polarity == EPAPER_POLARITY_BLACK_IS_1) {
        bit_value = !white;
    }
    set_native_frame_bit(driver, frame, nx, ny, bit_value);
}

static bool build_native_mono_frame(struct EpaperDriver *driver, uint8_t *frame,
    uint8_t *line_buf, BaseDisplayItem *items, int items_len, bool draw_bw_plane)
{
    size_t frame_len = mono_frame_bytes(driver);
    memset(frame, driver->desc->layout.polarity == EPAPER_POLARITY_WHITE_IS_1 ? 0xFF : 0x00,
        frame_len);

    for (int vy = 0; vy < driver->mono_screen.h; vy++) {
        if (draw_bw_plane) {
            render_mono_line_raw(driver, line_buf, vy, items, items_len);
        } else {
            memset(line_buf, 0xFF, view_mono_line_bytes(driver));
        }

        for (int vx = 0; vx < driver->mono_screen.w; vx++) {
            bool white = ((line_buf[vx / 8] >> (vx % 8)) & 0x01) != 0;
            int nx;
            int ny;
            if (!epaper_view_to_native(driver->desc, vx, vy, &nx, &ny)) {
                return false;
            }
            set_native_mono_pixel(driver, frame, nx, ny, white);
        }
    }

    return true;
}

static bool write_mono_plane(struct EpaperDriver *driver, uint8_t *frame_buf,
    uint8_t *line_buf,
    BaseDisplayItem *items, int items_len, bool draw_bw_plane,
    uint8_t *capture_frame)
{
    if (!build_native_mono_frame(driver, frame_buf, line_buf,
            items, items_len, draw_bw_plane)) {
        ESP_LOGE(TAG, "Failed to transform logical e-paper frame into native RAM.");
        return false;
    }

    if (capture_frame != NULL) {
        memcpy(capture_frame, frame_buf, mono_frame_bytes(driver));
    }

    write_mono_frame(driver, frame_buf);
    return true;
}

static uint8_t get_epaper_packed_pixel(uint8_t *line_buf, int xpos)
{
    uint8_t packed = line_buf[xpos / 2];
    if ((xpos & 1) == 0) {
        return packed >> 4;
    }
    return packed & 0x0F;
}

static bool build_native_4gray_plane(struct EpaperDriver *driver,
    uint8_t *gray_buf, uint8_t *plane_buf, BaseDisplayItem *items,
    int items_len, int plane_bit)
{
    int gray_bytes = (driver->mono_screen.w + 1) / 2;
    memset(plane_buf, 0x00, mono_frame_bytes(driver));

    for (int vy = 0; vy < driver->mono_screen.h; vy++) {
        memset(gray_buf, 0x00, gray_bytes);
        int xpos = 0;
        while (xpos < driver->mono_screen.w) {
            int drawn_pixels = epaper_draw_4gray_x(&driver->screen, gray_buf, xpos, vy, items, items_len);
            xpos += drawn_pixels;
        }

        for (xpos = 0; xpos < driver->mono_screen.w; xpos++) {
            uint8_t gray = get_epaper_packed_pixel(gray_buf, xpos);
            bool bit_value;
            if (driver->desc->controller == EPAPER_CONTROLLER_UC8276) {
                // UC8276 4-gray expects inverted two-bit gray planes:
                // plane 0 carries the high bit, plane 1 carries the low bit.
                bit_value = (gray & (plane_bit == 0 ? 0x02 : 0x01)) == 0;
            } else {
                bit_value = (gray & (1 << plane_bit)) != 0;
            }
            if (bit_value) {
                int nx;
                int ny;
                if (!epaper_view_to_native(driver->desc, xpos, vy, &nx, &ny)) {
                    return false;
                }
                set_native_frame_bit(driver, plane_buf, nx, ny, true);
            }
        }
    }

    return true;
}

static bool write_4gray_plane(struct EpaperDriver *driver,
    uint8_t *gray_buf, uint8_t *plane_buf, BaseDisplayItem *items,
    int items_len, int plane_bit)
{
    if (!build_native_4gray_plane(driver, gray_buf, plane_buf,
            items, items_len, plane_bit)) {
        ESP_LOGE(TAG, "Failed to transform logical 4-gray e-paper frame into native RAM.");
        return false;
    }

    write_mono_frame(driver, plane_buf);
    return true;
}

struct EPaperProgramRun
{
    uint8_t *plane_buf;
    uint8_t *line_buf;
    uint8_t *gray_buf;
    BaseDisplayItem *items;
    int items_len;
    bool capture_next_plane;
    bool is_4gray;
};

struct EPaperProgramDriverContext
{
    struct EpaperDriver *driver;
    struct EPaperProgramRun *run;
};

static void epaper_program_command(void *ctx, uint8_t command,
    const uint8_t *data, size_t len)
{
    struct EPaperProgramDriverContext *program_ctx = ctx;
    spi_dc_write_cmd_data(&program_ctx->driver->bus, command, data, len);
}

static void epaper_program_delay_ms(void *ctx, uint8_t delay_ms)
{
    (void) ctx;
    epaper_delay_ms(delay_ms);
}

static void epaper_program_reset(void *ctx, uint8_t high_ms,
    uint8_t low_ms, uint8_t settle_ms)
{
    struct EPaperProgramDriverContext *program_ctx = ctx;
    struct EpaperDriver *driver = program_ctx->driver;
    gpio_set_level(driver->reset_gpio, 1);
    epaper_delay_ms(high_ms);
    gpio_set_level(driver->reset_gpio, 0);
    epaper_delay_ms(low_ms);
    gpio_set_level(driver->reset_gpio, 1);
    epaper_delay_ms(settle_ms);
}

static bool epaper_program_wait_busy(void *ctx, uint8_t level,
    uint16_t timeout_ms)
{
    struct EPaperProgramDriverContext *program_ctx = ctx;
    return wait_busy_timeout(program_ctx->driver, level, timeout_ms);
}

static bool epaper_program_insert_plane(void *ctx, uint8_t plane_id)
{
    struct EPaperProgramDriverContext *program_ctx = ctx;
    struct EpaperDriver *driver = program_ctx->driver;
    struct EPaperProgramRun *run = program_ctx->run;

    if (run == NULL || run->plane_buf == NULL) {
        ESP_LOGE(TAG, "INSERT_PLANE used without render state.");
        return false;
    }

    uint8_t *capture_frame = NULL;
    if (run->capture_next_plane) {
        if (!ensure_prev_frame(driver)) {
            return false;
        }
        capture_frame = driver->prev_frame;
        run->capture_next_plane = false;
    }

    switch (plane_id) {
        case 0:
            if ((driver->desc->controller == EPAPER_CONTROLLER_SSD16XX
                    || driver->desc->controller == EPAPER_CONTROLLER_UC8276)
                && run->is_4gray) {
                if (run->gray_buf == NULL) {
                    return false;
                }
                return write_4gray_plane(driver, run->gray_buf, run->plane_buf,
                    run->items, run->items_len, 0);
            }
            if (run->line_buf == NULL) {
                return false;
            }
            return write_mono_plane(driver, run->plane_buf, run->line_buf,
                run->items, run->items_len, true, capture_frame);
        case 1:
            if ((driver->desc->controller == EPAPER_CONTROLLER_SSD16XX
                    || driver->desc->controller == EPAPER_CONTROLLER_UC8276)
                && run->is_4gray) {
                if (run->gray_buf == NULL) {
                    return false;
                }
                return write_4gray_plane(driver, run->gray_buf, run->plane_buf,
                    run->items, run->items_len, 1);
            }
            if (run->line_buf == NULL) {
                return false;
            }
            return write_mono_plane(driver, run->plane_buf, run->line_buf,
                run->items, run->items_len, false, capture_frame);
        default:
            ESP_LOGE(TAG, "Unsupported INSERT_PLANE id %d.", plane_id);
            return false;
    }
}

static bool epaper_program_insert_lut(void *ctx, uint8_t slot_id)
{
    if (slot_id >= EPAPER_MAX_LUT_SLOTS) {
        return false;
    }
    struct EPaperProgramDriverContext *program_ctx = ctx;
    struct EpaperDriver *driver = program_ctx->driver;
    if (driver->desc->lut_slots[slot_id].bytes == NULL
        || driver->desc->lut_slots[slot_id].len == 0) {
        ESP_LOGE(TAG, "INSERT_LUT references missing LUT slot %d.", slot_id);
        return false;
    }
    return epaper_write_lut_bytes(driver,
        driver->desc->lut_slots[slot_id].bytes,
        driver->desc->lut_slots[slot_id].len);
}

static bool epaper_program_insert_prev_frame(void *ctx, uint8_t plane_id)
{
    (void) plane_id;
    struct EPaperProgramDriverContext *program_ctx = ctx;
    struct EpaperDriver *driver = program_ctx->driver;
    if (!ensure_prev_frame(driver)) {
        return false;
    }
    write_mono_frame(driver, driver->prev_frame);
    return true;
}

static bool epaper_program_capture_frame(void *ctx)
{
    struct EPaperProgramDriverContext *program_ctx = ctx;
    if (program_ctx->run == NULL) {
        return false;
    }
    program_ctx->run->capture_next_plane = true;
    return true;
}

static bool epaper_program_mark_prev_valid(void *ctx)
{
    struct EPaperProgramDriverContext *program_ctx = ctx;
    program_ctx->driver->state.prev_valid = true;
    return true;
}

static bool epaper_run_driver_program(struct EpaperDriver *driver,
    const struct EPaperProgram *program, struct EPaperProgramRun *run)
{
    struct EPaperProgramDriverContext program_ctx = {
        .driver = driver,
        .run = run
    };
    const struct EPaperProgramOps ops = {
        .ctx = &program_ctx,
        .command = epaper_program_command,
        .delay_ms = epaper_program_delay_ms,
        .reset = epaper_program_reset,
        .wait_busy = epaper_program_wait_busy,
        .insert_plane = epaper_program_insert_plane,
        .insert_lut = epaper_program_insert_lut,
        .insert_prev_frame = epaper_program_insert_prev_frame,
        .capture_frame = epaper_program_capture_frame,
        .mark_prev_valid = epaper_program_mark_prev_valid
    };
    return epaper_run_program(program, &ops);
}

static void epaper_forget_previous_frame(struct EpaperDriver *driver)
{
    driver->state.prev_valid = false;
    driver->state.fast_refresh_count = 0;
    driver->state.needs_reseed = false;
}

static void epaper_clear_after_wake_policy(struct EpaperDriver *driver)
{
    driver->state.after_wake_pending = false;
    driver->state.after_wake_can_reseed_prev = false;
    driver->state.after_wake_refresh = EPAPER_AFTER_WAKE_REFRESH_ALLOW;
}

static void epaper_reset_refresh_history(struct EpaperDriver *driver)
{
    driver->state.last_refresh_mode = EPAPER_REFRESH_FULL;
    driver->state.fast_refresh_count = 0;
    driver->state.needs_reseed = false;
}

static const struct EPaperSleepMode *epaper_find_sleep_mode(struct EpaperDriver *driver,
    term mode_term)
{
    if (!term_is_atom(mode_term)) {
        return NULL;
    }
    uint32_t atom_index = (uint32_t) term_to_atom_index(mode_term);
    for (int i = 0; i < driver->desc->sleep_mode_count; i++) {
        const struct EPaperSleepMode *mode = &driver->desc->sleep_modes[i];
        if (mode->atom_index == atom_index) {
            return mode;
        }
    }
    return NULL;
}

static bool epaper_run_init_sequence(struct EpaperDriver *driver, bool reset_first)
{
    if (reset_first) {
        display_reset(driver);
        wait_busy_level(driver, driver->desc->busy_idle_level);
    }
    if (driver->desc->init.bytes != NULL) {
        return epaper_run_driver_program(driver, &driver->desc->init, NULL);
    }
    if (driver->desc->init_seq != NULL) {
        epaper_execute_init_seq(&driver->bus, driver->busy_gpio,
            driver->desc->init_seq, driver->desc->init_seq_len,
            driver->desc->init_wait_busy_between_cmds);
        return true;
    }

    if (!reset_first) {
        display_reset(driver);
        wait_busy_level(driver, driver->desc->busy_idle_level);
    }
    return true;
}

static bool epaper_run_wake_sequence(struct EpaperDriver *driver,
    const struct EPaperSleepMode *mode)
{
    if (mode != NULL && mode->wake_policy == EPAPER_WAKE_PROGRAM) {
        return epaper_run_driver_program(driver, &mode->wake, NULL);
    }
    if (mode != NULL && mode->wake_policy == EPAPER_WAKE_INIT) {
        return epaper_run_init_sequence(driver, false);
    }
    return epaper_run_init_sequence(driver, true);
}

static bool epaper_wake_if_asleep(struct EpaperDriver *driver)
{
    if (!driver->state.asleep) {
        return true;
    }

    const struct EPaperSleepMode *mode = driver->state.asleep_mode;
    if (!epaper_run_wake_sequence(driver, mode)) {
        ESP_LOGW(TAG, "E-paper wake sequence failed for %s.", driver->desc->name);
        return false;
    }

    driver->state.asleep = false;
    driver->state.asleep_mode = NULL;
    bool prev_valid_before_wake = driver->state.prev_valid;
    bool host_prev_preserved = mode != NULL
        && mode->host_prev_frame == EPAPER_HOST_PREV_FRAME_PRESERVE;
    bool controller_ram_retained = mode != NULL
        && mode->controller_ram == EPAPER_CONTROLLER_RAM_RETAINED;
    if (!(host_prev_preserved && controller_ram_retained)) {
        driver->state.prev_valid = false;
    }
    epaper_reset_refresh_history(driver);
    driver->state.after_wake_pending = true;
    driver->state.after_wake_can_reseed_prev =
        host_prev_preserved && prev_valid_before_wake;
    driver->state.after_wake_refresh = mode != NULL
        ? mode->after_wake_refresh
        : EPAPER_AFTER_WAKE_REFRESH_FULL;
    return true;
}

static bool do_sleep(Context *ctx, term mode_term)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);

    if (driver->state.asleep) {
        return true;
    }

    const struct EPaperSleepMode *mode = epaper_find_sleep_mode(driver, mode_term);
    if (mode == NULL) {
        ESP_LOGI(TAG, "E-paper descriptor for %s has no requested sleep mode.",
            driver->desc->name);
        return false;
    }

    if (!epaper_run_driver_program(driver, &mode->enter, NULL)) {
        ESP_LOGW(TAG, "E-paper sleep program failed for %s.", driver->desc->name);
        return false;
    }

    driver->state.asleep = true;
    driver->state.asleep_mode = mode;
    epaper_clear_after_wake_policy(driver);
    if (mode->host_prev_frame == EPAPER_HOST_PREV_FRAME_INVALIDATE) {
        epaper_forget_previous_frame(driver);
    }
    return true;
}

static bool do_wake(Context *ctx)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);

    if (driver->state.asleep) {
        return epaper_wake_if_asleep(driver);
    }
    return true;
}

static const struct EPaperProgram *epaper_program_for_refresh_mode(
    struct EpaperDriver *driver, enum EPaperRefreshMode mode)
{
    switch (mode) {
        case EPAPER_REFRESH_FULL:
            return &driver->desc->program_full;
        case EPAPER_REFRESH_FAST:
            return &driver->desc->program_fast;
        case EPAPER_REFRESH_PARTIAL:
            return &driver->desc->program_partial;
        case EPAPER_REFRESH_4GRAY:
            return &driver->desc->program_4gray;
        default:
            return &driver->desc->program_full;
    }
}

static uint8_t epaper_refresh_mode_bit(enum EPaperRefreshMode mode)
{
    return (uint8_t) (1U << mode);
}

static bool epaper_refresh_mode_allowed(const struct EPaperDesc *desc,
    enum EPaperRefreshMode mode)
{
    return (desc->refresh_mode_mask & epaper_refresh_mode_bit(mode)) != 0;
}

static term epaper_refresh_mode_atom(Context *ctx, enum EPaperRefreshMode mode)
{
    switch (mode) {
        case EPAPER_REFRESH_FULL:
            return context_make_atom(ctx, ATOM_STR("\x4", "full"));
        case EPAPER_REFRESH_FAST:
            return context_make_atom(ctx, ATOM_STR("\x4", "fast"));
        case EPAPER_REFRESH_PARTIAL:
            return context_make_atom(ctx, ATOM_STR("\x7", "partial"));
        case EPAPER_REFRESH_4GRAY:
            return context_make_atom(ctx, ATOM_STR("\x5", "4gray"));
        default:
            return context_make_atom(ctx, ATOM_STR("\x7", "unknown"));
    }
}

static term epaper_controller_atom(Context *ctx, enum EPaperController controller)
{
    switch (controller) {
        case EPAPER_CONTROLLER_SSD16XX:
            return context_make_atom(ctx, ATOM_STR("\x7", "ssd16xx"));
        case EPAPER_CONTROLLER_JD79656:
            return context_make_atom(ctx, ATOM_STR("\x7", "jd79656"));
        case EPAPER_CONTROLLER_UC8151:
            return context_make_atom(ctx, ATOM_STR("\x6", "uc8151"));
        case EPAPER_CONTROLLER_UC8276:
            return context_make_atom(ctx, ATOM_STR("\x6", "uc8276"));
        case EPAPER_CONTROLLER_UC8175:
            return context_make_atom(ctx, ATOM_STR("\x6", "uc8175"));
        case EPAPER_CONTROLLER_ACEP7:
            return context_make_atom(ctx, ATOM_STR("\x5", "acep7"));
        default:
            return context_make_atom(ctx, ATOM_STR("\x7", "unknown"));
    }
}

static bool epaper_program_inserts_prev_frame(const struct EPaperProgram *program)
{
    if (program == NULL || program->bytes == NULL) {
        return false;
    }

    size_t pos = 0;
    while (pos + 2 <= program->len) {
        uint8_t opcode = program->bytes[pos++];
        uint8_t flags_len = program->bytes[pos++];
        size_t data_len = flags_len & EPAPER_PROGRAM_LEN_MASK;
        bool is_meta = (flags_len & EPAPER_PROGRAM_META) != 0;
        bool has_delay = (flags_len & EPAPER_PROGRAM_DELAY) != 0;
        size_t next = pos + data_len + (has_delay ? 1 : 0);
        if (next > program->len) {
            return false;
        }
        if (is_meta && opcode == EPAPER_PROGRAM_INSERT_PREV_FRAME) {
            return true;
        }
        pos = next;
    }

    return false;
}

static void epaper_mark_refresh_success(struct EpaperDriver *driver,
    enum EPaperRefreshMode resolved_mode)
{
    epaper_clear_after_wake_policy(driver);
    driver->state.last_refresh_mode = resolved_mode;
    if (resolved_mode == EPAPER_REFRESH_FULL || resolved_mode == EPAPER_REFRESH_4GRAY) {
        driver->state.fast_refresh_count = 0;
        driver->state.needs_reseed = false;
    } else {
        driver->state.fast_refresh_count++;
    }
}

static const struct EPaperProgram *epaper_resolve_refresh_program(struct EpaperDriver *driver,
    enum EPaperRefreshMode requested_mode, enum EPaperRefreshMode *resolved_mode)
{
    enum EPaperRefreshMode mode = requested_mode;
    const struct EPaperProgram *after_wake_program = epaper_program_for_refresh_mode(driver, mode);
    bool after_wake_reseed_allowed =
        driver->state.after_wake_pending
        && driver->state.after_wake_refresh
            == EPAPER_AFTER_WAKE_REFRESH_ALLOW_IF_PROGRAM_RESEEDS
        && driver->state.after_wake_can_reseed_prev
        && epaper_program_inserts_prev_frame(after_wake_program);

    // Promote differential refreshes to full when the previous-frame state is not usable.
    bool differential_mode = mode == EPAPER_REFRESH_FAST || mode == EPAPER_REFRESH_PARTIAL;
    if (differential_mode) {
        if ((!driver->state.prev_valid && !after_wake_reseed_allowed)
            || driver->state.needs_reseed) {
            ESP_LOGI(TAG, "Promoting refresh to FULL: prev_valid=%d, needs_reseed=%d",
                driver->state.prev_valid, driver->state.needs_reseed);
            mode = EPAPER_REFRESH_FULL;
        } else if (driver->state.last_refresh_mode != EPAPER_REFRESH_FULL
                   && driver->state.last_refresh_mode != mode) {
            ESP_LOGI(TAG, "Promoting refresh to FULL: mode transition from %d to %d",
                driver->state.last_refresh_mode, mode);
            mode = EPAPER_REFRESH_FULL;
        } else if (driver->desc->max_fast_refreshes > 0
                   && driver->state.fast_refresh_count >= driver->desc->max_fast_refreshes) {
            ESP_LOGI(TAG, "Promoting refresh to FULL: fast refresh count %d reached limit %d",
                driver->state.fast_refresh_count, driver->desc->max_fast_refreshes);
            mode = EPAPER_REFRESH_FULL;
        }
    }

    if ((mode == EPAPER_REFRESH_FAST || mode == EPAPER_REFRESH_PARTIAL)
        && driver->state.after_wake_pending) {
        if (driver->state.after_wake_refresh == EPAPER_AFTER_WAKE_REFRESH_FULL) {
            ESP_LOGI(TAG, "Promoting refresh to FULL: after-wake policy requires full refresh.");
            mode = EPAPER_REFRESH_FULL;
        } else if (driver->state.after_wake_refresh
                == EPAPER_AFTER_WAKE_REFRESH_ALLOW_IF_PROGRAM_RESEEDS
            && !after_wake_reseed_allowed) {
            ESP_LOGI(TAG,
                "Promoting refresh to FULL: after-wake program does not reseed previous frame.");
            mode = EPAPER_REFRESH_FULL;
        }
    }

    ESP_LOGI(TAG, "Resolved refresh program: requested=%d, last=%d, resolved=%d",
        requested_mode, driver->state.last_refresh_mode, mode);

    *resolved_mode = mode;
    return epaper_program_for_refresh_mode(driver, mode);
}

static bool do_update_descriptor_mono(struct EpaperDriver *driver,
    BaseDisplayItem *items, int items_len, enum EPaperRefreshMode requested_mode)
{
    enum EPaperRefreshMode resolved_mode;
    const struct EPaperProgram *program = epaper_resolve_refresh_program(driver, requested_mode, &resolved_mode);

    if (program->bytes == NULL) {
        ESP_LOGE(TAG, "E-paper descriptor for %s has no refresh program for mode %d.",
            driver->desc->name, resolved_mode);
        return false;
    }

    const size_t frame_len = mono_frame_bytes(driver);
    const int line_bytes = view_mono_line_bytes(driver);
    uint8_t *frame_buf = heap_caps_malloc(frame_len, MALLOC_CAP_DMA);
    uint8_t *line_buf = heap_caps_malloc(line_bytes, MALLOC_CAP_DMA);
    if (UNLIKELY(!frame_buf || !line_buf)) {
        fprintf(stderr, "do_update: failed to alloc e-paper frame buffers\n");
        free(frame_buf);
        free(line_buf);
        return false;
    }

    struct EPaperProgramRun run = {
        .plane_buf = frame_buf,
        .line_buf = line_buf,
        .gray_buf = NULL,
        .items = items,
        .items_len = items_len,
        .capture_next_plane = false,
        .is_4gray = false
    };

    bool ok = epaper_run_driver_program(driver, program, &run);
    if (!ok) {
        ESP_LOGW(TAG, "E-paper refresh program failed for %s.", driver->desc->name);
    } else {
        epaper_mark_refresh_success(driver, resolved_mode);
    }

    free(frame_buf);
    free(line_buf);
    return ok;
}

static bool do_update_descriptor_4gray(struct EpaperDriver *driver,
    BaseDisplayItem *items, int items_len, enum EPaperRefreshMode requested_mode)
{
    enum EPaperRefreshMode resolved_mode;
    const struct EPaperProgram *program = epaper_resolve_refresh_program(driver, requested_mode, &resolved_mode);

    if (program->bytes == NULL) {
        ESP_LOGE(TAG, "E-paper descriptor for %s has no 4-gray refresh program.",
            driver->desc->name);
        return false;
    }

    const int gray_bytes = (driver->mono_screen.w + 1) / 2;
    const size_t frame_len = mono_frame_bytes(driver);
    const int line_bytes = view_mono_line_bytes(driver);
    bool is_4gray = (resolved_mode == EPAPER_REFRESH_4GRAY);
    uint8_t *gray_buf = NULL;
    if (is_4gray) {
        gray_buf = heap_caps_malloc(gray_bytes, MALLOC_CAP_DMA);
        if (UNLIKELY(!gray_buf)) {
            fprintf(stderr, "do_update: failed to alloc 4-gray buffer\n");
            return false;
        }
    }
    uint8_t *plane_buf = heap_caps_malloc(frame_len, MALLOC_CAP_DMA);
    uint8_t *line_buf = heap_caps_malloc(line_bytes, MALLOC_CAP_DMA);
    if (UNLIKELY(!plane_buf || !line_buf)) {
        fprintf(stderr, "do_update: failed to alloc plane buffers\n");
        free(gray_buf);
        free(plane_buf);
        free(line_buf);
        return false;
    }

    struct EPaperProgramRun run = {
        .plane_buf = plane_buf,
        .line_buf = line_buf,
        .gray_buf = gray_buf,
        .items = items,
        .items_len = items_len,
        .capture_next_plane = false,
        .is_4gray = is_4gray
    };

    bool ok = epaper_run_driver_program(driver, program, &run);
    if (!ok) {
        ESP_LOGW(TAG, "E-paper 4-gray refresh program failed for %s.",
            driver->desc->name);
    } else {
        epaper_mark_refresh_success(driver, resolved_mode);
    }

    free(gray_buf);
    free(plane_buf);
    free(line_buf);
    return ok;
}

static bool do_update(Context *ctx, term display_list, term update_opts)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);
    if (!epaper_wake_if_asleep(driver)) {
        return false;
    }

    if (driver->desc->controller == EPAPER_CONTROLLER_ACEP7) {
        maybe_refresh(ctx);
        wait_some_time(ctx);
    }

    BaseDisplayItem *items;
    size_t len;
    if (UNLIKELY(display_items_new_list(display_list, &items, &len, ctx) != DisplayItemsOk)) {
        return false;
    }

    // Resolve update refresh mode
    enum EPaperRefreshMode refresh_mode = driver->desc->default_refresh;
    if (update_opts != term_nil()) {
        term mode_term = interop_kv_get_value_default(update_opts, ATOM_STR("\x7", "refresh"), term_nil(), ctx->global);
        if (mode_term != term_nil()) {
            enum EPaperRefreshMode parsed_mode;
            if (epaper_parse_refresh_mode(mode_term, ctx, &parsed_mode)) {
                if (epaper_refresh_mode_allowed(driver->desc, parsed_mode)) {
                    refresh_mode = parsed_mode;
                } else {
                    ESP_LOGW(TAG,
                        "Refresh mode is not declared for %s; using default.",
                        driver->desc->name);
                }
            } else {
                ESP_LOGE(TAG, "Invalid refresh mode option specified in update.");
            }
        }
    }

    if (driver->desc->controller != EPAPER_CONTROLLER_ACEP7) {
        bool ok;
        if (driver->desc->palette_size == 4) {
            ok = do_update_descriptor_4gray(driver, items, len, refresh_mode);
        } else {
            ok = do_update_descriptor_mono(driver, items, len, refresh_mode);
        }
        display_items_delete(items, len);
        if (ok) {
            update_last_refresh_ts(ctx);
        }
        return ok;
    }

    send_frame_preamble(driver);

    // DTM — data transfer to panel memory.
    spi_dc_write_command(&driver->bus, 0x10);

    int screen_width = driver->screen.w;
    int screen_height = driver->screen.h;

    uint8_t *buf = heap_caps_malloc(screen_width / 2, MALLOC_CAP_DMA);
    if (UNLIKELY(!buf)) {
        fprintf(stderr, "do_update: failed to alloc buf\n");
        display_items_delete(items, len);
        return false;
    }
    memset(buf, 0x11, screen_width / 2);

    bool transaction_in_progress = false;

    spi_device_acquire_bus(driver->bus.spi_disp.handle, portMAX_DELAY);

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = epaper_draw_x(&driver->screen, buf, xpos, ypos, items, len);
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
    epaper_clear_after_wake_policy(driver);

    display_items_delete(items, len);

    update_last_refresh_ts(ctx);
    return true;
}

static term epaper_info_kv(Context *ctx, Heap *heap, AtomString key,
    term value, term tail)
{
    term item = term_alloc_tuple(2, heap);
    term_put_tuple_element(item, 0, context_make_atom(ctx, key));
    term_put_tuple_element(item, 1, value);
    return term_list_prepend(item, tail, heap);
}

static term epaper_refresh_modes_term(Context *ctx, Heap *heap,
    const struct EPaperDesc *desc)
{
    term modes = term_nil();
    for (int mode = EPAPER_REFRESH_MODE_COUNT - 1; mode >= 0; mode--) {
        if (epaper_refresh_mode_allowed(desc, (enum EPaperRefreshMode) mode)) {
            modes = term_list_prepend(
                epaper_refresh_mode_atom(ctx, (enum EPaperRefreshMode) mode),
                modes, heap);
        }
    }
    return modes;
}

static term epaper_sleep_modes_term(Heap *heap, const struct EPaperDesc *desc)
{
    term modes = term_nil();
    for (int i = desc->sleep_mode_count - 1; i >= 0; i--) {
        modes = term_list_prepend(
            term_from_atom_index((atom_index_t) desc->sleep_modes[i].atom_index),
            modes, heap);
    }
    return modes;
}

static term epaper_build_info(Context *ctx, struct EpaperDriver *driver,
    Heap *heap)
{
    const struct EPaperDesc *desc = driver->desc;
    term info = term_nil();
    info = epaper_info_kv(ctx, heap, ATOM_STR("\x6", "asleep"),
        driver->state.asleep ? TRUE_ATOM : FALSE_ATOM, info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\xB", "sleep_modes"),
        epaper_sleep_modes_term(heap, desc), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\xF", "default_refresh"),
        epaper_refresh_mode_atom(ctx, desc->default_refresh), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\xD", "refresh_modes"),
        epaper_refresh_modes_term(ctx, heap, desc), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\xC", "palette_size"),
        term_from_int(desc->palette_size), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\x8", "rotation"),
        term_from_int(desc->rotation), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\xD", "native_height"),
        term_from_int(desc->native_height), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\xC", "native_width"),
        term_from_int(desc->native_width), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\x6", "height"),
        term_from_int(desc->view_height), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\x5", "width"),
        term_from_int(desc->view_width), info);
    info = epaper_info_kv(ctx, heap, ATOM_STR("\xA", "controller"),
        epaper_controller_atom(ctx, desc->controller), info);
    return info;
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
    int req_arity = term_get_tuple_arity(req);
    term cmd = term_get_tuple_element(req, 0);
    term reply_status = OK_ATOM;
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {

        bool update_ok = false;
        if (req_arity < 2) {
            ESP_LOGE(TAG, "Invalid update request: expected {update, Items} or {update, Items, Opts}.");
        } else {
            term display_list = term_get_tuple_element(req, 1);
            term update_opts = term_nil();
            if (req_arity >= 3) {
                update_opts = term_get_tuple_element(req, 2);
            }
            update_ok = do_update(ctx, display_list, update_opts);
        }
        driver->state.last_op_ok = update_ok;

        // Reply already sent at enqueue time by
        // try_pre_ack_render_cmd in display_task.c. Sending another
        // reply here would leak a stray `{Ref, ok}' into the caller's
        // mailbox after port:call has already returned.
        return;

    } else if (cmd == context_make_atom(ctx, "\x5" "sleep")) {
        if (req_arity != 2) {
            ESP_LOGE(TAG, "Invalid sleep request: expected {sleep, Mode}.");
            reply_status = ERROR_ATOM;
        } else {
            reply_status = do_sleep(ctx, term_get_tuple_element(req, 1))
                ? OK_ATOM
                : ERROR_ATOM;
        }

    } else if (cmd == context_make_atom(ctx, "\x4" "wake")) {
        reply_status = do_wake(ctx) ? OK_ATOM : ERROR_ATOM;

    } else if (cmd == context_make_atom(ctx, "\x9" "wait_idle")) {
        reply_status = driver->state.last_op_ok ? OK_ATOM : ERROR_ATOM;

    } else if (cmd == context_make_atom(ctx, "\x4" "info")
            || cmd == context_make_atom(ctx, "\xC" "capabilities")) {
        BEGIN_WITH_STACK_HEAP(
            TUPLE_SIZE(2) + REF_SIZE
                + 11 * (TUPLE_SIZE(2) + CONS_SIZE)
                + (EPAPER_REFRESH_MODE_COUNT + EPAPER_MAX_SLEEP_MODES) * CONS_SIZE
                + 8,
            heap);
        term return_tuple = term_alloc_tuple(2, &heap);
        term_put_tuple_element(return_tuple, 0, gen_message.ref);
        term_put_tuple_element(return_tuple, 1, epaper_build_info(ctx, driver, &heap));
        display_message_send(gen_message.pid, return_tuple, ctx->global);
        END_WITH_STACK_HEAP(heap, ctx->global);
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
    term_put_tuple_element(return_tuple, 1, reply_status);

    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void clear_screen_internal(Context *ctx, int color,
    bool clear_after_wake_pending)
{
    struct EpaperDriver *driver = EPAPER_DRIVER_FROM_CTX(ctx);
    if (!epaper_wake_if_asleep(driver)) {
        return;
    }

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
    if (clear_after_wake_pending) {
        epaper_clear_after_wake_policy(driver);
    }
}

static bool epaper_term_bool(term value, bool *out)
{
    if (value == TRUE_ATOM) {
        *out = true;
        return true;
    }
    if (value == FALSE_ATOM) {
        *out = false;
        return true;
    }
    return false;
}

static bool epaper_parse_int_field(term container, Context *ctx,
    AtomString key, int *out)
{
    term value = interop_kv_get_value_default(
        container, key, term_nil(), ctx->global);
    if (value == term_nil()) {
        return true;
    }
    if (term_is_integer(value)) {
        *out = term_to_int(value);
        return true;
    }
    return false;
}

static bool epaper_parse_bool_field(term container, Context *ctx,
    AtomString key, bool *out)
{
    term value = interop_kv_get_value_default(
        container, key, term_nil(), ctx->global);
    if (value == term_nil()) {
        return true;
    }
    bool parsed;
    if (epaper_term_bool(value, &parsed)) {
        *out = parsed;
        return true;
    }
    return false;
}

static bool epaper_copy_binary_field(term container, Context *ctx,
    AtomString key, uint8_t **owned_bytes, const uint8_t **bytes, size_t *len)
{
    term value = interop_kv_get_value_default(
        container, key, term_nil(), ctx->global);
    if (value == term_nil()) {
        *owned_bytes = NULL;
        *bytes = NULL;
        *len = 0;
        return true;
    }

    if (!term_is_binary(value)) {
        return false;
    }

    size_t value_len = term_binary_size(value);
    uint8_t *copy = malloc(value_len == 0 ? 1 : value_len);
    if (copy == NULL) {
        return false;
    }

    memcpy(copy, term_binary_data(value), value_len);
    *owned_bytes = copy;
    *bytes = copy;
    *len = value_len;
    return true;
}

static bool epaper_copy_binary_term(term value,
    uint8_t **owned_bytes, const uint8_t **bytes, size_t *len)
{
    if (!term_is_binary(value)) {
        return false;
    }

    size_t value_len = term_binary_size(value);
    uint8_t *copy = malloc(value_len == 0 ? 1 : value_len);
    if (copy == NULL) {
        return false;
    }

    memcpy(copy, term_binary_data(value), value_len);
    *owned_bytes = copy;
    *bytes = copy;
    *len = value_len;
    return true;
}

static void epaper_free_descriptor_override(struct EpaperDriver *driver)
{
    free(driver->term_desc_name);
    driver->term_desc_name = NULL;
    free(driver->term_init_seq_bytes);
    driver->term_init_seq_bytes = NULL;
    free(driver->term_frame_preamble_seq_bytes);
    driver->term_frame_preamble_seq_bytes = NULL;

    for (int i = 0; i < EPAPER_TERM_PROGRAM_COUNT; i++) {
        free(driver->term_program_bytes[i]);
        driver->term_program_bytes[i] = NULL;
    }

    for (int i = 0; i < EPAPER_MAX_SLEEP_MODES; i++) {
        free(driver->term_sleep_enter_bytes[i]);
        driver->term_sleep_enter_bytes[i] = NULL;
        free(driver->term_sleep_wake_bytes[i]);
        driver->term_sleep_wake_bytes[i] = NULL;
    }

    for (int i = 0; i < EPAPER_MAX_LUT_SLOTS; i++) {
        free(driver->term_lut_bytes[i]);
        driver->term_lut_bytes[i] = NULL;
    }
}

static void epaper_free_init_failed_driver(struct EpaperDriver *driver,
    bool spi_device_added)
{
    if (driver == NULL) {
        return;
    }

    if (driver->ctx != NULL && driver->ctx->platform_data == &driver->display_args) {
        driver->ctx->platform_data = NULL;
    }

    if (driver->display_args.messages_queue != NULL) {
        vQueueDelete(driver->display_args.messages_queue);
        driver->display_args.messages_queue = NULL;
    }

    if (spi_device_added && driver->bus.spi_disp.handle != NULL) {
        spi_bus_remove_device(driver->bus.spi_disp.handle);
        driver->bus.spi_disp.handle = NULL;
    }

    free(driver->prev_frame);
    driver->prev_frame = NULL;
    epaper_free_descriptor_override(driver);
    free(driver);
}

static bool epaper_parse_controller(term val, Context *ctx, enum EPaperController *out)
{
    if (val == context_make_atom(ctx, ATOM_STR("\x7", "ssd16xx"))) {
        *out = EPAPER_CONTROLLER_SSD16XX;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x7", "jd79656"))) {
        *out = EPAPER_CONTROLLER_JD79656;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x6", "uc8151"))) {
        *out = EPAPER_CONTROLLER_UC8151;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x6", "uc8276"))) {
        *out = EPAPER_CONTROLLER_UC8276;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x6", "uc8175"))) {
        *out = EPAPER_CONTROLLER_UC8175;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x5", "acep7"))) {
        *out = EPAPER_CONTROLLER_ACEP7;
        return true;
    }
    return false;
}

static bool epaper_parse_layout(term val, Context *ctx, struct EPaperFrameLayout *out)
{
    if (val == context_make_atom(ctx, ATOM_STR("\x7", "row_msb"))) {
        out->byte_order = EPAPER_BYTE_ORDER_ROW_MAJOR;
        out->bit_order = EPAPER_BIT_ORDER_MSB_LEFT;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x7", "row_lsb"))) {
        out->byte_order = EPAPER_BYTE_ORDER_ROW_MAJOR;
        out->bit_order = EPAPER_BIT_ORDER_LSB_LEFT;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\xA", "column_msb"))) {
        out->byte_order = EPAPER_BYTE_ORDER_COLUMN_MAJOR;
        out->bit_order = EPAPER_BIT_ORDER_MSB_LEFT;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\xA", "column_lsb"))) {
        out->byte_order = EPAPER_BYTE_ORDER_COLUMN_MAJOR;
        out->bit_order = EPAPER_BIT_ORDER_LSB_LEFT;
        return true;
    }
    return false;
}

static bool epaper_parse_polarity(term val, Context *ctx, enum EPaperPolarity *out)
{
    if (val == context_make_atom(ctx, ATOM_STR("\x7", "white_1"))) {
        *out = EPAPER_POLARITY_WHITE_IS_1;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x7", "black_1"))) {
        *out = EPAPER_POLARITY_BLACK_IS_1;
        return true;
    }
    return false;
}

static bool epaper_parse_refresh_mode(term val, Context *ctx, enum EPaperRefreshMode *out)
{
    if (val == context_make_atom(ctx, ATOM_STR("\x4", "full"))) {
        *out = EPAPER_REFRESH_FULL;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x4", "fast"))) {
        *out = EPAPER_REFRESH_FAST;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x7", "partial"))) {
        *out = EPAPER_REFRESH_PARTIAL;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x5", "4gray"))) {
        *out = EPAPER_REFRESH_4GRAY;
        return true;
    }
    return false;
}

static bool epaper_parse_refresh_modes(term modes, Context *ctx, uint8_t *out)
{
    if (!term_is_nonempty_list(modes)) {
        return false;
    }

    uint8_t mask = 0;
    term t = modes;
    while (term_is_nonempty_list(t)) {
        enum EPaperRefreshMode mode;
        if (!epaper_parse_refresh_mode(term_get_list_head(t), ctx, &mode)) {
            return false;
        }
        uint8_t bit = epaper_refresh_mode_bit(mode);
        if ((mask & bit) != 0) {
            return false;
        }
        mask |= bit;
        t = term_get_list_tail(t);
    }

    if (t != term_nil() || (mask & EPAPER_REFRESH_MODE_FULL) == 0) {
        return false;
    }

    *out = mask;
    return true;
}

static bool epaper_parse_controller_ram_policy(term val, Context *ctx,
    enum EPaperControllerRamPolicy *out)
{
    if (val == context_make_atom(ctx, ATOM_STR("\x7", "unknown"))) {
        *out = EPAPER_CONTROLLER_RAM_UNKNOWN;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x8", "retained"))) {
        *out = EPAPER_CONTROLLER_RAM_RETAINED;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x4", "lost"))) {
        *out = EPAPER_CONTROLLER_RAM_LOST;
        return true;
    }
    return false;
}

static bool epaper_parse_host_prev_frame_policy(term val, Context *ctx,
    enum EPaperHostPrevFramePolicy *out)
{
    if (val == context_make_atom(ctx, ATOM_STR("\x8", "preserve"))) {
        *out = EPAPER_HOST_PREV_FRAME_PRESERVE;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\xA", "invalidate"))) {
        *out = EPAPER_HOST_PREV_FRAME_INVALIDATE;
        return true;
    }
    return false;
}

static bool epaper_parse_after_wake_refresh_policy(term val, Context *ctx,
    enum EPaperAfterWakeRefreshPolicy *out)
{
    if (val == context_make_atom(ctx, ATOM_STR("\x5", "allow"))) {
        *out = EPAPER_AFTER_WAKE_REFRESH_ALLOW;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x4", "full"))) {
        *out = EPAPER_AFTER_WAKE_REFRESH_FULL;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\x18", "allow_if_program_reseeds"))) {
        *out = EPAPER_AFTER_WAKE_REFRESH_ALLOW_IF_PROGRAM_RESEEDS;
        return true;
    }
    return false;
}

static bool epaper_parse_palette(term val, Context *ctx,
    const uint8_t (**palette)[3], int *palette_size)
{
    if (val == context_make_atom(ctx, ATOM_STR("\x5", "acep7"))
        || val == context_make_atom(ctx, ATOM_STR("\x6", "acep7c"))) {
        *palette = epaper_acep_palette;
        *palette_size = 7;
        return true;
    }
    if (val == context_make_atom(ctx, ATOM_STR("\xA", "gdep073e01"))) {
        *palette = epaper_gdep073e01_palette;
        *palette_size = 7;
        return true;
    }
    return false;
}

static bool epaper_parse_sleep_modes(struct EpaperDriver *driver,
    term descriptor, Context *ctx)
{
    term sleep_modes = interop_kv_get_value_default(
        descriptor, ATOM_STR("\xB", "sleep_modes"), term_nil(), ctx->global);
    if (sleep_modes == term_nil()) {
        driver->term_desc.sleep_mode_count = 0;
        return true;
    }

    int mode_count = 0;
    term t = sleep_modes;
    while (term_is_nonempty_list(t)) {
        if (mode_count >= EPAPER_MAX_SLEEP_MODES) {
            ESP_LOGE(TAG, "Too many e-paper sleep modes; maximum is %d.",
                EPAPER_MAX_SLEEP_MODES);
            return false;
        }

        term entry = term_get_list_head(t);
        if (!term_is_tuple(entry) || term_get_tuple_arity(entry) != 2) {
            ESP_LOGE(TAG, "Invalid e-paper sleep mode entry.");
            return false;
        }

        term mode_atom = term_get_tuple_element(entry, 0);
        term props = term_get_tuple_element(entry, 1);
        if (!term_is_atom(mode_atom)
            || (props != term_nil() && !term_is_nonempty_list(props))) {
            ESP_LOGE(TAG, "Invalid e-paper sleep mode name or properties.");
            return false;
        }
        uint32_t mode_atom_index = (uint32_t) term_to_atom_index(mode_atom);
        for (int i = 0; i < mode_count; i++) {
            if (driver->term_desc.sleep_modes[i].atom_index == mode_atom_index) {
                ESP_LOGE(TAG, "Duplicate e-paper sleep mode.");
                return false;
            }
        }

        struct EPaperSleepMode *mode = &driver->term_desc.sleep_modes[mode_count];
        memset(mode, 0, sizeof(*mode));
        mode->atom_index = mode_atom_index;
        mode->wake_policy = EPAPER_WAKE_RESET_INIT;
        mode->controller_ram = EPAPER_CONTROLLER_RAM_UNKNOWN;
        mode->host_prev_frame = EPAPER_HOST_PREV_FRAME_INVALIDATE;
        mode->after_wake_refresh = EPAPER_AFTER_WAKE_REFRESH_FULL;

        term enter = interop_kv_get_value_default(
            props, ATOM_STR("\x5", "enter"), term_nil(), ctx->global);
        if (enter == term_nil()
            || !epaper_copy_binary_term(enter,
                &driver->term_sleep_enter_bytes[mode_count],
                &mode->enter.bytes, &mode->enter.len)) {
            ESP_LOGE(TAG, "Invalid or missing e-paper sleep mode enter program.");
            return false;
        }

        term wake = interop_kv_get_value_default(
            props, ATOM_STR("\x4", "wake"), term_nil(), ctx->global);
        if (wake != term_nil()) {
            if (term_is_binary(wake)) {
                if (!epaper_copy_binary_term(wake,
                        &driver->term_sleep_wake_bytes[mode_count],
                        &mode->wake.bytes, &mode->wake.len)) {
                    ESP_LOGE(TAG, "Invalid e-paper sleep mode wake program.");
                    return false;
                }
                mode->wake_policy = EPAPER_WAKE_PROGRAM;
            } else if (wake == context_make_atom(ctx, ATOM_STR("\x4", "init"))) {
                mode->wake_policy = EPAPER_WAKE_INIT;
            } else if (wake == context_make_atom(ctx, ATOM_STR("\xA", "reset_init"))) {
                mode->wake_policy = EPAPER_WAKE_RESET_INIT;
            } else {
                ESP_LOGE(TAG, "Invalid e-paper sleep mode wake policy.");
                return false;
            }
        }

        term controller_ram = interop_kv_get_value_default(
            props, ATOM_STR("\xE", "controller_ram"), term_nil(), ctx->global);
        if (controller_ram != term_nil()
            && !epaper_parse_controller_ram_policy(controller_ram, ctx, &mode->controller_ram)) {
            ESP_LOGE(TAG, "Invalid e-paper sleep mode controller_ram policy.");
            return false;
        }

        term host_prev_frame = interop_kv_get_value_default(
            props, ATOM_STR("\xF", "host_prev_frame"), term_nil(), ctx->global);
        if (host_prev_frame != term_nil()
            && !epaper_parse_host_prev_frame_policy(host_prev_frame, ctx,
                &mode->host_prev_frame)) {
            ESP_LOGE(TAG, "Invalid e-paper sleep mode host_prev_frame policy.");
            return false;
        }

        term after_wake_refresh = interop_kv_get_value_default(
            props, ATOM_STR("\x12", "after_wake_refresh"), term_nil(), ctx->global);
        if (after_wake_refresh != term_nil()
            && !epaper_parse_after_wake_refresh_policy(after_wake_refresh, ctx,
                &mode->after_wake_refresh)) {
            ESP_LOGE(TAG, "Invalid e-paper sleep mode after_wake_refresh policy.");
            return false;
        }

        mode_count++;
        t = term_get_list_tail(t);
    }

    if (t != term_nil()) {
        ESP_LOGE(TAG, "Invalid e-paper sleep_modes list.");
        return false;
    }

    driver->term_desc.sleep_mode_count = mode_count;
    return true;
}

static bool epaper_validate_descriptor_refresh_modes(const struct EPaperDesc *desc)
{
    if ((desc->refresh_mode_mask & epaper_refresh_mode_bit(desc->default_refresh)) == 0) {
        ESP_LOGE(TAG, "default_refresh is not listed in refresh_modes.");
        return false;
    }

    if (desc->controller == EPAPER_CONTROLLER_ACEP7) {
        if (desc->refresh_mode_mask != EPAPER_REFRESH_MODE_FULL) {
            ESP_LOGE(TAG, "ACeP e-paper descriptors currently support only full refresh.");
            return false;
        }
        return true;
    }

    for (int mode = 0; mode < EPAPER_REFRESH_MODE_COUNT; mode++) {
        if (!epaper_refresh_mode_allowed(desc, (enum EPaperRefreshMode) mode)) {
            continue;
        }
        const struct EPaperProgram *program;
        switch ((enum EPaperRefreshMode) mode) {
            case EPAPER_REFRESH_FULL:
                program = &desc->program_full;
                break;
            case EPAPER_REFRESH_FAST:
                program = &desc->program_fast;
                break;
            case EPAPER_REFRESH_PARTIAL:
                program = &desc->program_partial;
                break;
            case EPAPER_REFRESH_4GRAY:
                program = &desc->program_4gray;
                break;
            default:
                program = NULL;
                break;
        }
        if (program->bytes == NULL) {
            ESP_LOGE(TAG, "refresh_modes declares a mode without a program.");
            return false;
        }
    }

    return true;
}

static bool epaper_validate_descriptor_geometry(const struct EPaperDesc *desc)
{
    if (desc->native_width <= 0 || desc->native_height <= 0
        || desc->view_width <= 0 || desc->view_height <= 0) {
        ESP_LOGE(TAG, "Invalid e-paper geometry: native=%dx%d view=%dx%d.",
            desc->native_width, desc->native_height,
            desc->view_width, desc->view_height);
        return false;
    }

    if (desc->controller == EPAPER_CONTROLLER_ACEP7
        && (desc->rotation != 0
            || desc->view_width != desc->native_width
            || desc->view_height != desc->native_height)) {
        ESP_LOGE(TAG,
            "ACeP e-paper descriptors only support native orientation: native=%dx%d view=%dx%d rotation=%d.",
            desc->native_width, desc->native_height,
            desc->view_width, desc->view_height, desc->rotation);
        return false;
    }

    switch (desc->rotation) {
        case 0:
        case 180:
            if (desc->view_width == desc->native_width
                && desc->view_height == desc->native_height) {
                return true;
            }
            break;
        case 90:
        case 270:
            if (desc->view_width == desc->native_height
                && desc->view_height == desc->native_width) {
                return true;
            }
            break;
        default:
            ESP_LOGE(TAG, "Invalid e-paper rotation %d; expected 0, 90, 180, or 270.",
                desc->rotation);
            return false;
    }

    ESP_LOGE(TAG, "Invalid e-paper geometry for rotation %d: native=%dx%d view=%dx%d.",
        desc->rotation, desc->native_width, desc->native_height,
        desc->view_width, desc->view_height);
    return false;
}

static bool epaper_parse_descriptor_override(struct EpaperDriver *driver,
    term opts, Context *ctx)
{
    term descriptor = interop_kv_get_value_default(
        opts, ATOM_STR("\xA", "descriptor"), term_nil(), ctx->global);
    if (descriptor == term_nil()) {
        return true;
    }
    if (!term_is_nonempty_list(descriptor)) {
        ESP_LOGE(TAG, "Invalid e-paper descriptor: expected keyword list.");
        return false;
    }

    memset(driver->term_program_bytes, 0, sizeof(driver->term_program_bytes));
    memset(driver->term_sleep_enter_bytes, 0, sizeof(driver->term_sleep_enter_bytes));
    memset(driver->term_sleep_wake_bytes, 0, sizeof(driver->term_sleep_wake_bytes));
    memset(driver->term_lut_bytes, 0, sizeof(driver->term_lut_bytes));
    driver->term_init_seq_bytes = NULL;
    driver->term_frame_preamble_seq_bytes = NULL;
    driver->term_desc_name = NULL;
    driver->term_desc = *driver->desc;
    memset(driver->term_desc.sleep_modes, 0, sizeof(driver->term_desc.sleep_modes));
    driver->term_desc.sleep_mode_count = 0;

    // Check version first
    int descriptor_version = 0;
    if (!epaper_parse_int_field(descriptor, ctx,
            ATOM_STR("\x12", "descriptor_version"),
            &descriptor_version)) {
        ESP_LOGE(TAG, "Invalid e-paper descriptor_version.");
        return false;
    }
    driver->term_desc.descriptor_version = descriptor_version;
    if (driver->term_desc.descriptor_version != 3) {
        ESP_LOGE(TAG, "Unsupported e-paper descriptor version %d. Only version 3 is supported.",
            driver->term_desc.descriptor_version);
        return false;
    }

    term name = interop_kv_get_value_default(
        descriptor, ATOM_STR("\x4", "name"), term_nil(), ctx->global);
    if (name != term_nil()) {
        int name_ok;
        char *parsed_name = interop_term_to_string(name, &name_ok);
        if (name_ok && parsed_name != NULL) {
            driver->term_desc_name = parsed_name;
            driver->term_desc.name = parsed_name;
        } else {
            free(parsed_name);
        }
    }

    // Geometry
    if (!epaper_parse_int_field(descriptor, ctx, ATOM_STR("\xC", "native_width"),
            &driver->term_desc.native_width)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\xD", "native_height"),
            &driver->term_desc.native_height)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\xA", "view_width"),
            &driver->term_desc.view_width)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\xB", "view_height"),
            &driver->term_desc.view_height)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\x8", "rotation"),
            &driver->term_desc.rotation)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\xC", "spi_clock_hz"),
            &driver->term_desc.spi_clock_hz)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\xF", "busy_idle_level"),
            &driver->term_desc.busy_idle_level)) {
        ESP_LOGE(TAG, "Invalid integer field in e-paper descriptor.");
        return false;
    }

    if (!epaper_parse_bool_field(descriptor, ctx, ATOM_STR("\x10", "use_gpio_pullups"),
            &driver->term_desc.use_gpio_pullups)) {
        ESP_LOGE(TAG, "Invalid e-paper descriptor use_gpio_pullups.");
        return false;
    }

    if (!epaper_parse_int_field(descriptor, ctx, ATOM_STR("\xC", "palette_size"),
            &driver->term_desc.palette_size)) {
        ESP_LOGE(TAG, "Invalid e-paper descriptor palette_size.");
        return false;
    }

    term palette_term = interop_kv_get_value_default(
        descriptor, ATOM_STR("\x7", "palette"), term_nil(), ctx->global);
    if (palette_term != term_nil()) {
        const uint8_t (*palette)[3];
        int palette_size;
        if (!epaper_parse_palette(palette_term, ctx, &palette, &palette_size)) {
            ESP_LOGE(TAG, "Invalid palette in e-paper descriptor.");
            return false;
        }
        driver->term_desc.palette = palette;
        driver->term_desc.palette_size = palette_size;
    }

    // Parse Controller
    term controller_term = interop_kv_get_value_default(descriptor, ATOM_STR("\xA", "controller"), term_nil(), ctx->global);
    if (controller_term != term_nil()) {
        enum EPaperController ctrl;
        if (!epaper_parse_controller(controller_term, ctx, &ctrl)) {
            ESP_LOGE(TAG, "Invalid controller in descriptor.");
            return false;
        }
        driver->term_desc.controller = ctrl;
    }

    // Parse Frame Layout
    term layout_term = interop_kv_get_value_default(descriptor, ATOM_STR("\xC", "frame_layout"), term_nil(), ctx->global);
    if (layout_term != term_nil()) {
        struct EPaperFrameLayout layout;
        if (!epaper_parse_layout(layout_term, ctx, &layout)) {
            ESP_LOGE(TAG, "Invalid frame_layout in descriptor.");
            return false;
        }
        driver->term_desc.layout = layout;
    }

    // Parse Polarity
    term polarity_term = interop_kv_get_value_default(descriptor, ATOM_STR("\x8", "polarity"), term_nil(), ctx->global);
    if (polarity_term != term_nil()) {
        enum EPaperPolarity polarity;
        if (!epaper_parse_polarity(polarity_term, ctx, &polarity)) {
            ESP_LOGE(TAG, "Invalid polarity in descriptor.");
            return false;
        }
        driver->term_desc.layout.polarity = polarity;
    }

    // Parse Refresh Modes
    term refresh_modes_term = interop_kv_get_value_default(
        descriptor, ATOM_STR("\xD", "refresh_modes"), term_nil(), ctx->global);
    if (refresh_modes_term == term_nil()
        || !epaper_parse_refresh_modes(refresh_modes_term, ctx,
            &driver->term_desc.refresh_mode_mask)) {
        ESP_LOGE(TAG, "Invalid refresh_modes in descriptor.");
        return false;
    }

    // Parse Default Refresh
    term default_refresh_term = interop_kv_get_value_default(descriptor, ATOM_STR("\xF", "default_refresh"), term_nil(), ctx->global);
    if (default_refresh_term != term_nil()) {
        enum EPaperRefreshMode default_refresh;
        if (!epaper_parse_refresh_mode(default_refresh_term, ctx, &default_refresh)) {
            ESP_LOGE(TAG, "Invalid default_refresh mode in descriptor.");
            return false;
        }
        driver->term_desc.default_refresh = default_refresh;
    }
    if (!epaper_refresh_mode_allowed(&driver->term_desc,
            driver->term_desc.default_refresh)) {
        ESP_LOGE(TAG, "default_refresh is not listed in refresh_modes.");
        return false;
    }

    // Parse Programs Map
    term programs_map = interop_kv_get_value_default(descriptor, ATOM_STR("\x8", "programs"), term_nil(), ctx->global);
    if (programs_map != term_nil()) {
        if (!epaper_copy_binary_field(programs_map, ctx, ATOM_STR("\x4", "init"),
                &driver->term_program_bytes[0],
                &driver->term_desc.init.bytes, &driver->term_desc.init.len)
            || !epaper_copy_binary_field(programs_map, ctx, ATOM_STR("\x4", "full"),
                &driver->term_program_bytes[1],
                &driver->term_desc.program_full.bytes, &driver->term_desc.program_full.len)
            || !epaper_copy_binary_field(programs_map, ctx, ATOM_STR("\x4", "fast"),
                &driver->term_program_bytes[2],
                &driver->term_desc.program_fast.bytes, &driver->term_desc.program_fast.len)
            || !epaper_copy_binary_field(programs_map, ctx, ATOM_STR("\x7", "partial"),
                &driver->term_program_bytes[3],
                &driver->term_desc.program_partial.bytes, &driver->term_desc.program_partial.len)
            || !epaper_copy_binary_field(programs_map, ctx, ATOM_STR("\x5", "4gray"),
                &driver->term_program_bytes[4],
                &driver->term_desc.program_4gray.bytes, &driver->term_desc.program_4gray.len)) {
            ESP_LOGE(TAG, "Invalid e-paper descriptor program binary.");
            return false;
        }
    }

    if (!epaper_validate_descriptor_refresh_modes(&driver->term_desc)) {
        return false;
    }

    if (!epaper_parse_sleep_modes(driver, descriptor, ctx)) {
        return false;
    }

    if (!epaper_copy_binary_field(descriptor, ctx, ATOM_STR("\x8", "init_seq"),
            &driver->term_init_seq_bytes,
            &driver->term_desc.init_seq, &driver->term_desc.init_seq_len)
        || !epaper_copy_binary_field(descriptor, ctx, ATOM_STR("\x12", "frame_preamble_seq"),
            &driver->term_frame_preamble_seq_bytes,
            &driver->term_desc.frame_preamble_seq, &driver->term_desc.frame_preamble_seq_len)) {
        ESP_LOGE(TAG, "Invalid e-paper ACeP sequence binary.");
        return false;
    }

    int refresh_data_byte = driver->term_desc.refresh_data_byte;
    if (!epaper_parse_bool_field(descriptor, ctx, ATOM_STR("\x1B", "init_wait_busy_between_cmds"),
            &driver->term_desc.init_wait_busy_between_cmds)
        || !epaper_parse_bool_field(descriptor, ctx, ATOM_STR("\x10", "refresh_has_data"),
            &driver->term_desc.refresh_has_data)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\x11", "refresh_data_byte"),
            &refresh_data_byte)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\x19", "post_power_off_busy_level"),
            &driver->term_desc.post_power_off_busy_level)
        || !epaper_parse_int_field(descriptor, ctx, ATOM_STR("\x19", "periodic_refresh_interval"),
            &driver->term_desc.periodic_refresh_interval)) {
        ESP_LOGE(TAG, "Invalid ACeP e-paper descriptor field.");
        return false;
    }
    if (refresh_data_byte < 0 || refresh_data_byte > UINT8_MAX) {
        ESP_LOGE(TAG, "Invalid refresh_data_byte in e-paper descriptor.");
        return false;
    }
    driver->term_desc.refresh_data_byte = (uint8_t) refresh_data_byte;

    // Parse Timing Map
    term timing_map = interop_kv_get_value_default(descriptor, ATOM_STR("\x6", "timing"), term_nil(), ctx->global);
    if (timing_map != term_nil()) {
        if (!epaper_parse_int_field(timing_map, ctx, ATOM_STR("\x10", "full_expected_ms"), &driver->term_desc.full_expected_ms)
            || !epaper_parse_int_field(timing_map, ctx, ATOM_STR("\x10", "fast_expected_ms"), &driver->term_desc.fast_expected_ms)
            || !epaper_parse_int_field(timing_map, ctx, ATOM_STR("\x10", "poll_interval_ms"), &driver->term_desc.poll_interval_ms)
            || !epaper_parse_int_field(timing_map, ctx, ATOM_STR("\n", "timeout_ms"), &driver->term_desc.timeout_ms)) {
            ESP_LOGE(TAG, "Invalid field in e-paper descriptor timing.");
            return false;
        }
    }

    // Parse Ghosting Map
    term ghosting_map = interop_kv_get_value_default(descriptor, ATOM_STR("\x8", "ghosting"), term_nil(), ctx->global);
    if (ghosting_map != term_nil()) {
        if (!epaper_parse_int_field(ghosting_map, ctx, ATOM_STR("\x12", "max_fast_refreshes"), &driver->term_desc.max_fast_refreshes)
            || !epaper_parse_bool_field(ghosting_map, ctx, ATOM_STR("\x11", "reseed_on_timeout"), &driver->term_desc.reseed_on_timeout)) {
            ESP_LOGE(TAG, "Invalid field in e-paper descriptor ghosting.");
            return false;
        }
    }

    // Parse LUTs
    if (!epaper_copy_binary_field(descriptor, ctx, ATOM_STR("\x8", "lut_full"),
            &driver->term_lut_bytes[EPAPER_LUT_SLOT_FULL],
            &driver->term_desc.lut_slots[EPAPER_LUT_SLOT_FULL].bytes,
            &driver->term_desc.lut_slots[EPAPER_LUT_SLOT_FULL].len)
        || !epaper_copy_binary_field(descriptor, ctx, ATOM_STR("\xB", "lut_partial"),
            &driver->term_lut_bytes[EPAPER_LUT_SLOT_PARTIAL],
            &driver->term_desc.lut_slots[EPAPER_LUT_SLOT_PARTIAL].bytes,
            &driver->term_desc.lut_slots[EPAPER_LUT_SLOT_PARTIAL].len)
        || !epaper_copy_binary_field(descriptor, ctx, ATOM_STR("\x9", "lut_4gray"),
            &driver->term_lut_bytes[EPAPER_LUT_SLOT_4GRAY],
            &driver->term_desc.lut_slots[EPAPER_LUT_SLOT_4GRAY].bytes,
            &driver->term_desc.lut_slots[EPAPER_LUT_SLOT_4GRAY].len)
        || !epaper_copy_binary_field(descriptor, ctx, ATOM_STR("\x8", "lut_fast"),
            &driver->term_lut_bytes[EPAPER_LUT_SLOT_FAST],
            &driver->term_desc.lut_slots[EPAPER_LUT_SLOT_FAST].bytes,
            &driver->term_desc.lut_slots[EPAPER_LUT_SLOT_FAST].len)) {
        ESP_LOGE(TAG, "Invalid e-paper descriptor LUT binary.");
        return false;
    }

    if (driver->term_desc.palette_size <= 0) {
        driver->term_desc.palette_size = 2;
    }

    // Setup 4-gray palette if relevant
    if ((driver->term_desc.controller == EPAPER_CONTROLLER_SSD16XX
            || driver->term_desc.controller == EPAPER_CONTROLLER_UC8276)
        && driver->term_desc.palette_size == 4) {
        driver->term_desc.palette = epaper_ssd1680_4gray_palette;
    }
    if (driver->term_desc.controller == EPAPER_CONTROLLER_ACEP7 && driver->term_desc.palette == NULL) {
        ESP_LOGE(TAG, "ACeP e-paper descriptor requires a known color palette.");
        return false;
    }
    if (driver->term_desc.controller == EPAPER_CONTROLLER_ACEP7
        && driver->term_desc.init.bytes == NULL
        && driver->term_desc.init_seq == NULL) {
        ESP_LOGE(TAG, "ACeP e-paper descriptor requires an init program or init_seq.");
        return false;
    }

    if (!epaper_validate_descriptor_geometry(&driver->term_desc)) {
        return false;
    }

    driver->desc = &driver->term_desc;
    ESP_LOGI(TAG, "Using Erlang e-paper descriptor: %s", driver->desc->name);
    return true;
}

static void display_spi_init(Context *ctx, term opts)
{
    term descriptor_term = interop_kv_get_value_default(
        opts, ATOM_STR("\xA", "descriptor"), term_nil(), ctx->global);

    if (descriptor_term == term_nil()) {
        ESP_LOGE(TAG, "Failed init: missing e-paper descriptor.");
        return;
    }

    const struct EPaperDesc *desc = &epaper_desc_term_default;

    struct EpaperDriver *driver = calloc(1, sizeof(struct EpaperDriver));
    if (UNLIKELY(!driver)) {
        ESP_LOGE(TAG, "Failed init: unable to allocate driver state.");
        return;
    }

    driver->desc = desc;
    driver->ctx = ctx;
    if (!epaper_parse_descriptor_override(driver, opts, ctx)) {
        ESP_LOGE(TAG, "Failed init: invalid descriptor override.");
        epaper_free_init_failed_driver(driver, false);
        return;
    }
    desc = driver->desc;
    if (!epaper_validate_descriptor_geometry(desc)) {
        ESP_LOGE(TAG, "Failed init: invalid descriptor geometry for '%s'.", desc->name);
        epaper_free_init_failed_driver(driver, false);
        return;
    }
    if (epaper_desc_requires_program(desc) && desc->program_full.bytes == NULL) {
        ESP_LOGE(TAG,
            "Failed init: e-paper descriptor '%s' requires a full refresh program.",
            desc->name);
        epaper_free_init_failed_driver(driver, false);
        return;
    }

    driver->screen.w = desc->view_width;
    driver->screen.h = desc->view_height;
    driver->screen.palette = desc->palette;
    driver->screen.palette_size = desc->palette_size;
    driver->mono_screen.w = desc->view_width;
    driver->mono_screen.h = desc->view_height;
    driver->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    if (driver->display_args.messages_queue == NULL) {
        ESP_LOGE(TAG, "Failed init: unable to allocate display queue.");
        epaper_free_init_failed_driver(driver, false);
        return;
    }
    driver->display_args.process_message_fn = process_message;
    driver->display_args.ctx = ctx;
    ctx->platform_data = &driver->display_args;

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.clock_speed_hz = desc->spi_clock_hz;
    if (!spi_display_parse_config(&spi_config, opts, ctx->global)) {
        ESP_LOGE(TAG, "Failed init: invalid SPI display configuration.");
        epaper_free_init_failed_driver(driver, false);
        return;
    }
    spi_display_init(&driver->bus.spi_disp, &spi_config);
    bool spi_device_added = true;

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x4", "busy"), &driver->busy_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &driver->bus.dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &driver->reset_gpio, ctx->global);
    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display GPIOs.");
        epaper_free_init_failed_driver(driver, spi_device_added);
        return;
    }

    gpio_set_direction(driver->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(driver->reset_gpio, 1);
    gpio_set_direction(driver->bus.dc_gpio, GPIO_MODE_OUTPUT);
    gpio_set_direction(driver->busy_gpio, GPIO_MODE_INPUT);
    if (desc->use_gpio_pullups) {
        gpio_set_pull_mode(driver->bus.dc_gpio, GPIO_PULLUP_ENABLE);
        gpio_set_pull_mode(driver->busy_gpio, GPIO_PULLUP_ENABLE);
        gpio_set_level(driver->bus.dc_gpio, 0);
    }

    driver->state.prev_valid = false;
    driver->state.fast_refresh_count = 0;
    driver->state.needs_reseed = false;
    driver->state.last_op_ok = true;
    driver->state.asleep = false;
    driver->state.asleep_mode = NULL;
    epaper_clear_after_wake_policy(driver);

    // Init sequence: init_list opt overrides the descriptor default.
    term init_list = interop_kv_get_value_default(
        opts, ATOM_STR("\x9", "init_list"), term_nil(), ctx->global);
    if (init_list != term_nil()) {
        display_reset(driver);
        wait_busy_level(driver, desc->busy_idle_level);
        display_init_using_list(driver, init_list);
    } else if (desc->init.bytes != NULL) {
        if (!epaper_run_driver_program(driver, &desc->init, NULL)) {
            ESP_LOGW(TAG, "E-paper descriptor init program failed for %s.",
                desc->name);
        }
    } else if (desc->init_seq != NULL) {
        display_reset(driver);
        wait_busy_level(driver, desc->busy_idle_level);
        epaper_execute_init_seq(&driver->bus, driver->busy_gpio,
            desc->init_seq, desc->init_seq_len,
            desc->init_wait_busy_between_cmds);
    }

    update_last_refresh_ts(ctx);
    driver->count_to_refresh = 0;

#if SELF_TEST
    for (int i = 0; i < 8; i++) {
        fprintf(stderr, "color: %i\n", i);
        clear_screen_internal(ctx, i, true);
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
    clear_screen_internal(ctx, 1, true);

    while (1)
        ;
#else
    if (xTaskCreate(display_task_process_messages, "display", 10000,
            &driver->display_args, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed init: unable to start display task.");
        epaper_free_init_failed_driver(driver, spi_device_added);
        return;
    }
#endif
}

Context *epaper_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_spi_init(ctx, opts);
    return ctx;
}

// Erlang-side init override
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
                epaper_delay_ms(term_to_int(data_term));
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
