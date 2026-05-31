/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 AtomGL contributors
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

#ifndef _EPAPER_PROGRAM_H_
#define _EPAPER_PROGRAM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EPAPER_MAX_LUT_SLOTS 4

#define EPAPER_PROGRAM_DELAY 0x80
#define EPAPER_PROGRAM_META 0x40
#define EPAPER_PROGRAM_LEN_MASK 0x3F
#define EPAPER_MAX_DESCRIPTOR_BINARY_LEN 4096

enum EPaperProgramOpcode
{
    EPAPER_PROGRAM_WAIT_BUSY = 0x01,
    EPAPER_PROGRAM_RESET = 0x02,
    EPAPER_PROGRAM_INSERT_PLANE = 0x03,
    EPAPER_PROGRAM_INSERT_LUT = 0x04,
    EPAPER_PROGRAM_INSERT_PREV_FRAME = 0x05,
    EPAPER_PROGRAM_CAPTURE_FRAME = 0x06,
    EPAPER_PROGRAM_MARK_PREV_VALID = 0x07,
    EPAPER_PROGRAM_LABEL = 0x10
};

enum EPaperLutSlot
{
    EPAPER_LUT_SLOT_FULL = 0,
    EPAPER_LUT_SLOT_PARTIAL = 1,
    EPAPER_LUT_SLOT_4GRAY = 2,
    EPAPER_LUT_SLOT_FAST = 3
};

struct EPaperProgram
{
    const uint8_t *bytes;
    size_t len;
};

struct EPaperLut
{
    const uint8_t *bytes;
    size_t len;
};

struct EPaperProgramOps
{
    void *ctx;

    void (*command)(void *ctx, uint8_t command, const uint8_t *data, size_t len);
    void (*delay_ms)(void *ctx, uint8_t delay_ms);
    void (*reset)(void *ctx, uint8_t high_ms, uint8_t low_ms, uint8_t settle_ms);

    bool (*wait_busy)(void *ctx, uint8_t level, uint16_t timeout_ms);
    bool (*insert_plane)(void *ctx, uint8_t plane_id);
    bool (*insert_lut)(void *ctx, uint8_t slot_id);
    bool (*insert_prev_frame)(void *ctx, uint8_t plane_id);
    bool (*capture_frame)(void *ctx);
    bool (*mark_prev_valid)(void *ctx);
};

bool epaper_run_program(const struct EPaperProgram *program,
    const struct EPaperProgramOps *ops);
bool epaper_validate_program(const struct EPaperProgram *program,
    bool allow_render_ops);

#endif
