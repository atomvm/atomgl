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

#include "epaper_program.h"

static bool epaper_run_meta_opcode(const struct EPaperProgramOps *ops,
    uint8_t opcode, const uint8_t *operands, uint8_t len)
{
    switch (opcode) {
        case EPAPER_PROGRAM_WAIT_BUSY:
            if (len != 3 || ops->wait_busy == NULL) {
                return false;
            }
            return ops->wait_busy(ops->ctx, operands[0],
                (uint16_t) operands[1] | ((uint16_t) operands[2] << 8));

        case EPAPER_PROGRAM_RESET:
            if (len != 3 || ops->reset == NULL) {
                return false;
            }
            ops->reset(ops->ctx, operands[0], operands[1], operands[2]);
            return true;

        case EPAPER_PROGRAM_INSERT_PLANE:
            if (len != 1 || ops->insert_plane == NULL) {
                return false;
            }
            return ops->insert_plane(ops->ctx, operands[0]);

        case EPAPER_PROGRAM_INSERT_LUT:
            if (len != 1 || ops->insert_lut == NULL) {
                return false;
            }
            return ops->insert_lut(ops->ctx, operands[0]);

        case EPAPER_PROGRAM_INSERT_PREV_FRAME:
            if (len != 1 || ops->insert_prev_frame == NULL) {
                return false;
            }
            return ops->insert_prev_frame(ops->ctx, operands[0]);

        case EPAPER_PROGRAM_CAPTURE_FRAME:
            if (len != 0 || ops->capture_frame == NULL) {
                return false;
            }
            return ops->capture_frame(ops->ctx);

        case EPAPER_PROGRAM_MARK_PREV_VALID:
            if (len != 0 || ops->mark_prev_valid == NULL) {
                return false;
            }
            return ops->mark_prev_valid(ops->ctx);

        case EPAPER_PROGRAM_LABEL:
            return len == 1;

        default:
            return false;
    }
}

bool epaper_run_program(const struct EPaperProgram *program,
    const struct EPaperProgramOps *ops)
{
    if (program == NULL || program->bytes == NULL) {
        return true;
    }
    if (ops == NULL || ops->command == NULL) {
        return false;
    }

    const uint8_t *pc = program->bytes;
    const uint8_t *end = program->bytes + program->len;
    while (pc < end) {
        if ((size_t) (end - pc) < 2) {
            return false;
        }

        uint8_t opcode = *pc++;
        uint8_t flags_len = *pc++;
        uint8_t len = flags_len & EPAPER_PROGRAM_LEN_MASK;
        if ((size_t) (end - pc) < len) {
            return false;
        }

        if (flags_len & EPAPER_PROGRAM_META) {
            if (!epaper_run_meta_opcode(ops, opcode, pc, len)) {
                return false;
            }
        } else {
            ops->command(ops->ctx, opcode, pc, len);
        }
        pc += len;

        if (flags_len & EPAPER_PROGRAM_DELAY) {
            if (pc >= end || ops->delay_ms == NULL) {
                return false;
            }
            ops->delay_ms(ops->ctx, *pc++);
        }
    }

    return true;
}
