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

bool epaper_validate_init_seq(const uint8_t *seq, size_t seq_len)
{
    if (seq == NULL) {
        return seq_len == 0;
    }
    if (seq_len > EPAPER_MAX_DESCRIPTOR_BINARY_LEN) {
        return false;
    }

    size_t pos = 0;
    while (pos < seq_len) {
        if (seq_len - pos < 2) {
            return false;
        }
        pos++;
        uint8_t flags_len = seq[pos++];
        uint8_t len = flags_len & 0x7F;

        if (seq_len - pos < len) {
            return false;
        }
        pos += len;

        if (flags_len & EPAPER_INIT_SEQ_DELAY) {
            if (seq_len - pos < 1) {
                return false;
            }
            pos++;
        }
    }
    return true;
}

bool epaper_execute_init_seq(struct SPIDCBus *bus, int busy_gpio,
    const uint8_t *seq, size_t seq_len, bool wait_busy_between_cmds)
{
    if (!epaper_validate_init_seq(seq, seq_len)) {
        return false;
    }

    size_t pos = 0;
    while (pos < seq_len) {
        uint8_t cmd = seq[pos++];
        uint8_t flags_len = seq[pos++];
        uint8_t len = flags_len & 0x7F;

        spi_dc_write_cmd_data(bus, cmd, seq + pos, len);
        pos += len;

        if (flags_len & EPAPER_INIT_SEQ_DELAY) {
            uint8_t delay_ms = seq[pos++];
            if (delay_ms > 0) {
                vTaskDelay(delay_ms_to_ticks(delay_ms));
            }
        }

        if (wait_busy_between_cmds) {
            wait_busy_high(busy_gpio);
        }
    }

    return true;
}
