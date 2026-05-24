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

#ifndef _EPAPER_COMMANDS_H_
#define _EPAPER_COMMANDS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "epaper_program.h"
#include "spi_dc_driver.h"

// --- Init sequence byte-array format ---
//
// Each entry:  [CMD] [FLAGS_LEN] [DATA_0 ... DATA_N] [DELAY_MS]
//   CMD:        command byte (any value, including 0x00)
//   FLAGS_LEN:  bits 6:0 = data byte count (0-127)
//               bit 7    = delay flag (DELAY_MS byte follows data)
//   DELAY_MS:   delay in milliseconds (0-255), present only if flag set
//
// The sequence is length-bounded: callers pass the array length as
// seq_len and the executor walks the buffer until that count is
// exhausted.  Length framing (rather than a sentinel byte) lets any
// command byte appear in an init sequence unambiguously — notably
// 0x00, which is PSR on most Waveshare / GoodDisplay controllers.

#define EPAPER_INIT_SEQ_DELAY 0x80

// Execute an init sequence over an SPI+DC bus.  seq/seq_len describe
// a byte array in the format documented above.  When
// wait_busy_between_cmds is true, a busy-high poll is inserted after
// each command (mirroring the Good Display / Waveshare convention of
// "command completes when BUSY rises").  busy_gpio is ignored when
// the flag is false.
void epaper_execute_init_seq(struct SPIDCBus *bus, int busy_gpio,
    const uint8_t *seq, size_t seq_len, bool wait_busy_between_cmds);

// Built-in init sequences.  Each array is paired with a size_t
// constant giving its length; callers pass both to
// epaper_execute_init_seq().
extern const uint8_t epaper_init_seq_acep7c[];
extern const size_t epaper_init_seq_acep7c_len;
extern const uint8_t epaper_init_seq_gdep073e01[];
extern const size_t epaper_init_seq_gdep073e01_len;

// --- Per-panel descriptor ---
//
enum EPaperController
{
    EPAPER_CONTROLLER_ACEP7,
    EPAPER_CONTROLLER_SSD16XX,
    EPAPER_CONTROLLER_JD79656,
    EPAPER_CONTROLLER_UC8175
};

enum EPaperByteOrder
{
    EPAPER_BYTE_ORDER_ROW_MAJOR,
    EPAPER_BYTE_ORDER_COLUMN_MAJOR
};

enum EPaperBitOrder
{
    EPAPER_BIT_ORDER_MSB_LEFT,
    EPAPER_BIT_ORDER_LSB_LEFT
};

enum EPaperPolarity
{
    EPAPER_POLARITY_WHITE_IS_1,
    EPAPER_POLARITY_BLACK_IS_1
};

enum EPaperCommandTarget
{
    EPAPER_COMMAND_TARGET_CURRENT_PLANE,
    EPAPER_COMMAND_TARGET_PREVIOUS_PLANE,
    EPAPER_COMMAND_TARGET_COLOR_PLANE
};

enum EPaperRefreshMode
{
    EPAPER_REFRESH_FULL,
    EPAPER_REFRESH_FAST,
    EPAPER_REFRESH_PARTIAL,
    EPAPER_REFRESH_4GRAY
};

struct EPaperFrameLayout
{
    enum EPaperByteOrder byte_order;
    enum EPaperBitOrder bit_order;
    enum EPaperPolarity polarity;
};

// Captures every panel-specific knob so a single unified driver can
// drive multiple controllers by compatible-string dispatch.

struct EPaperDesc
{
    const char *name;
    int native_width;
    int native_height;
    int view_width;
    int view_height;
    int rotation;
    int spi_clock_hz;

    // Color palette (RGB triplets) and its entry count.
    const uint8_t (*palette)[3];
    int palette_size;

    enum EPaperController controller;
    struct EPaperFrameLayout layout;
    enum EPaperCommandTarget command_target;

    // One-time init sequence (format documented above).
    const uint8_t *init_seq;
    size_t init_seq_len;
    bool init_wait_busy_between_cmds;

    bool use_gpio_pullups;

    // Optional per-frame preamble sent before DTM (0x10) on every
    // do_update and clear_screen.  NULL when unused.  Uses the same
    // byte-array format as init_seq and is executed without inter-
    // command BUSY polling.
    const uint8_t *frame_preamble_seq;
    size_t frame_preamble_seq_len;

    // Post-frame refresh protocol: PON (0x04) -> wait BUSY=1; DRF
    // (0x12) optionally followed by one data byte, then wait BUSY=1;
    // POF (0x02) then wait BUSY=post_power_off_busy_level.
    bool refresh_has_data;
    uint8_t refresh_data_byte;
    int post_power_off_busy_level;

    // BUSY pin level when the controller is idle / ready to receive
    // commands.  Used after reset (in display_spi_init) before
    // executing any init sequence.  Most Waveshare/Good Display PSR-
    // style controllers idle BUSY high (1); SSD1680 idles BUSY low
    // (0).  Get this wrong and the post-reset busy wait deadlocks.
    int busy_idle_level;

    // Periodic full-screen white-out.  Zero = disabled.  N > 0 means
    // "call clear_screen(7) once every N do_update() invocations" to
    // prevent ghosting on panels that need it (ACeP 5.65").
    int periodic_refresh_interval;

    // Data-driven descriptor programs.
    uint8_t descriptor_version;
    struct EPaperProgram init;
    struct EPaperProgram program_full;
    struct EPaperProgram program_fast;
    struct EPaperProgram program_partial;
    struct EPaperProgram program_4gray;
    struct EPaperProgram sleep;
    struct EPaperProgram wake;
    struct EPaperLut lut_slots[EPAPER_MAX_LUT_SLOTS];

    // Timing & ghosting fields
    int full_expected_ms;
    int fast_expected_ms;
    int poll_interval_ms;
    int timeout_ms;
    int max_fast_refreshes;
    bool reseed_on_timeout;

    enum EPaperRefreshMode default_refresh;
};

extern const struct EPaperDesc epaper_desc_acep7c;
extern const struct EPaperDesc epaper_desc_gdep073e01;

#endif
