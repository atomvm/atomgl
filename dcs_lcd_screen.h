/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2026 Davide Bettio <davide@uninstall.it>
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

#ifndef _DCS_LCD_SCREEN_H_
#define _DCS_LCD_SCREEN_H_

#include <stdint.h>

// Per-display state shared across the DCS LCD scanline rendering pipeline.
struct DCSLCDScreen
{
    int w;
    int h;
    int16_t x_offset;
    int16_t y_offset;
    // Scanlines per SPI transaction.
    int batch_lines;
    // Two buffers of batch_lines scanlines each.
    uint16_t *pixels;
    uint16_t *pixels_out;

    // ILI9488: 3 bytes/pixel.
    uint8_t *bytes;
    uint8_t *bytes_out;
};

#endif
