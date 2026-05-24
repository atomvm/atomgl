/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 Davide Bettio <davide@uninstall.it>
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

#include "epaper_commands.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>

#include "epaper_color.h"

static void wait_busy_high(int busy_gpio)
{
    while (gpio_get_level(busy_gpio) != 1) {
        vTaskDelay(100);
    }
}

static TickType_t delay_ms_to_ticks(uint32_t delay_ms)
{
    return (delay_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
}

void epaper_execute_init_seq(struct SPIDCBus *bus, int busy_gpio,
    const uint8_t *seq, size_t seq_len, bool wait_busy_between_cmds)
{
    const uint8_t *end = seq + seq_len;
    while (seq < end) {
        uint8_t cmd = *seq++;
        uint8_t flags_len = *seq++;
        uint8_t len = flags_len & 0x7F;

        spi_dc_write_cmd_data(bus, cmd, seq, len);
        seq += len;

        if (flags_len & EPAPER_INIT_SEQ_DELAY) {
            uint8_t delay_ms = *seq++;
            if (delay_ms > 0) {
                vTaskDelay(delay_ms_to_ticks(delay_ms));
            }
        }

        if (wait_busy_between_cmds) {
            wait_busy_high(busy_gpio);
        }
    }
}

// --- Built-in init sequences ---
//
// Transcribed mechanically from the original display_spi_init()
// bodies in 5in65_acep_7c_display_driver.c and
// gdep073e01_display_driver.c.  _Static_assert on sizeof guards
// against miscounted data bytes.

// clang-format off

// Waveshare 5.65" ACeP 7-color.  No BUSY polling between commands; a
// single 100 ms delay between VDCS (0x82) and the second CDI (0x50)
// replicates the vTaskDelay(10) at tick rate 100 Hz in the original
// driver.
const uint8_t epaper_init_seq_acep7c[] = {
    0x00, 2,                          0xEF, 0x08,                      // PSR
    0x01, 4,                          0x37, 0x00, 0x23, 0x23,          // PWRR
    0x03, 1,                          0x00,                            // POFS
    0x06, 3,                          0xC7, 0xC7, 0x1D,                // BTST
    0x30, 1,                          0x3C,                            // PLL
    0x40, 1,                          0x00,                            // TSE
    0x50, 1,                          0x3F,                            // CDI
    0x60, 1,                          0x22,                            // TCON
    0x61, 4,                          0x02, 0x58, 0x01, 0xC0,          // TRES (600x448)
    0xE3, 1,                          0xAA,                            // PWS
    0x82, EPAPER_INIT_SEQ_DELAY | 1,  0x80, 100,                       // VDCS + 100 ms
    0x50, 1,                          0x37,                            // CDI (second)
};
_Static_assert(sizeof(epaper_init_seq_acep7c) == 46,
    "epaper_init_seq_acep7c: miscounted bytes");
const size_t epaper_init_seq_acep7c_len = sizeof(epaper_init_seq_acep7c);

// Good Display GDEP073E01 7.3" 7-color.  BUSY polling interleaved
// after each command by the executor (wait_busy_between_cmds = true);
// no per-command delay bytes are required.  The leading 0xAA entry is
// an undocumented vendor preamble preserved verbatim.
const uint8_t epaper_init_seq_gdep073e01[] = {
    0xAA, 6,  0x49, 0x55, 0x20, 0x08, 0x09, 0x18,                      // vendor preamble
    0x01, 1,  0x3F,                                                    // PWRR
    0x00, 2,  0x5F, 0x69,                                              // PSR
    0x03, 4,  0x00, 0x54, 0x00, 0x44,                                  // POFS
    0x05, 4,  0x40, 0x1F, 0x1F, 0x2C,                                  // BTST1
    0x06, 4,  0x6F, 0x1F, 0x17, 0x49,                                  // BTST2
    0x08, 4,  0x6F, 0x1F, 0x1F, 0x22,                                  // BTST3
    0x30, 1,  0x00,                                                    // PLL
    0x50, 1,  0x3F,                                                    // CDI
    0x60, 2,  0x02, 0x00,                                              // TCON
    0x61, 4,  0x03, 0x20, 0x01, 0xE0,                                  // TRES (800x480)
    0x84, 1,  0x01,                                                    // T_VDCS
    0xE3, 1,  0x2F,                                                    // PWS
    0x04, 0,                                                           // PON
};
_Static_assert(sizeof(epaper_init_seq_gdep073e01) == 63,
    "epaper_init_seq_gdep073e01: miscounted bytes");
const size_t epaper_init_seq_gdep073e01_len = sizeof(epaper_init_seq_gdep073e01);

// --- Per-frame preambles ---
//
// ACeP 5.65" retransmits the resolution command (0x61 + 600x448)
// before every DTM; GoodDisplay's GDEP073E01 does not.  The preamble
// uses the same byte-array format as init_seq.

static const uint8_t epaper_preamble_acep7c[] = {
    0x61, 4,  0x02, 0x58, 0x01, 0xC0,                                  // TRES (600x448)
};
_Static_assert(sizeof(epaper_preamble_acep7c) == 6,
    "epaper_preamble_acep7c: miscounted bytes");

// --- Per-panel descriptors ---

const struct EPaperDesc epaper_desc_acep7c = {
    .name                        = "Waveshare 5.65\" ACeP 7-color",
    .native_width                = 600,
    .native_height               = 448,
    .view_width                  = 600,
    .view_height                 = 448,
    .rotation                    = 0,
    .spi_clock_hz                = 1000000,

    .palette                     = epaper_acep_palette,
    .palette_size                = 7,
    .controller                  = EPAPER_CONTROLLER_RAW,
    .layout                      = {
        .byte_order = EPAPER_BYTE_ORDER_ROW_MAJOR,
        .bit_order = EPAPER_BIT_ORDER_MSB_LEFT,
        .polarity = EPAPER_POLARITY_WHITE_IS_1
    },
    .command_target              = EPAPER_COMMAND_TARGET_COLOR_PLANE,

    .init_seq                    = epaper_init_seq_acep7c,
    .init_seq_len                = sizeof(epaper_init_seq_acep7c),
    .init_wait_busy_between_cmds = false,
    .use_gpio_pullups            = true,

    .frame_preamble_seq          = epaper_preamble_acep7c,
    .frame_preamble_seq_len      = sizeof(epaper_preamble_acep7c),

    .refresh_has_data            = false,
    .refresh_data_byte           = 0x00,
    .post_power_off_busy_level   = 0,
    .busy_idle_level             = 1,

    .periodic_refresh_interval   = 5,
};

const struct EPaperDesc epaper_desc_gdep073e01 = {
    .name                        = "Good Display GDEP073E01 7.3\" 7-color",
    .native_width                = 800,
    .native_height               = 480,
    .view_width                  = 800,
    .view_height                 = 480,
    .rotation                    = 0,
    .spi_clock_hz                = 4000000,

    .palette                     = epaper_gdep073e01_palette,
    .palette_size                = 7,
    .controller                  = EPAPER_CONTROLLER_RAW,
    .layout                      = {
        .byte_order = EPAPER_BYTE_ORDER_ROW_MAJOR,
        .bit_order = EPAPER_BIT_ORDER_MSB_LEFT,
        .polarity = EPAPER_POLARITY_WHITE_IS_1
    },
    .command_target              = EPAPER_COMMAND_TARGET_COLOR_PLANE,

    .init_seq                    = epaper_init_seq_gdep073e01,
    .init_seq_len                = sizeof(epaper_init_seq_gdep073e01),
    .init_wait_busy_between_cmds = true,
    .use_gpio_pullups            = true,

    .frame_preamble_seq          = NULL,
    .frame_preamble_seq_len      = 0,

    .refresh_has_data            = true,
    .refresh_data_byte           = 0x00,
    .post_power_off_busy_level   = 1,
    .busy_idle_level             = 1,

    .periodic_refresh_interval   = 0,
};

// clang-format on
