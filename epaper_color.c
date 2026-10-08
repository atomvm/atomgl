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

#include "epaper_color.h"

#include <stdbool.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

const uint8_t epaper_acep_palette[7][3] = {
    { 0x00, 0x00, 0x00 },
    { 0xFF, 0xFF, 0xFF },
    { 0x00, 0xFF, 0x00 },
    { 0x00, 0x00, 0xFF },
    { 0xFF, 0x00, 0x00 },
    { 0xFF, 0xFF, 0x00 },
    { 0xFF, 0x80, 0x00 }
};

const uint8_t epaper_gdep073e01_palette[7][3] = {
    { 0x19, 0x1E, 0x21 },
    { 0xE8, 0xE8, 0xE8 },
    { 0xEF, 0xDE, 0x44 },
    { 0xB2, 0x13, 0x18 },
    { 0xE8, 0xE8, 0xE8 },
    { 0x21, 0x57, 0xBA },
    { 0x12, 0x5F, 0x20 }
};

// App-facing SSD1680 4-gray palette. The plane writer maps these palette
// indices onto the SSD1680/Waveshare two-plane gray classes.
const uint8_t epaper_ssd1680_4gray_palette[4][3] = {
    { 0xFF, 0xFF, 0xFF },
    { 0xAA, 0xAA, 0xAA },
    { 0x55, 0x55, 0x55 },
    { 0x00, 0x00, 0x00 }
};

static inline float square(float p)
{
    return p * p;
}

static int clamp_u8(int v)
{
    if (v < 0) {
        return 0;
    }
    if (v > 255) {
        return 255;
    }
    return v;
}

static uint8_t nearest_palette_index(int r, int g, int b,
    const uint8_t palette[][3], int palette_size)
{
    float min = INT_MAX;
    int min_index = 0;

    for (int i = 0; i < palette_size; i++) {
        int r2 = palette[i][0];
        int g2 = palette[i][1];
        int b2 = palette[i][2];

#ifdef NO_WEIGHTS
        float d = square((r2 - r)) + square((g2 - g)) + square((b2 - b));
#else
        float d = square((r2 - r) * 0.30) + square((g2 - g) * 0.59) + square((b2 - b) * 0.11);
#endif

        if (d < min) {
            min = d;
            min_index = i;
        }
    }

    return min_index;
}

static bool exact_palette_index(uint8_t r, uint8_t g, uint8_t b,
    const uint8_t palette[][3], int palette_size, uint8_t *out)
{
    for (int i = 0; i < palette_size; i++) {
        if (palette[i][0] == r && palette[i][1] == g && palette[i][2] == b) {
            *out = i;
            return true;
        }
    }
    return false;
}

uint8_t epaper_dither_acep7(int x, int y, uint8_t r, uint8_t g, uint8_t b,
    const uint8_t palette[][3], int palette_size)
{
    uint8_t exact_index;
    if (exact_palette_index(r, g, b, palette, palette_size, &exact_index)) {
        return exact_index;
    }

    const uint8_t m[4][4] = {
        { 0, 8, 2, 10 },
        { 12, 4, 14, 6 },
        { 3, 11, 1, 9 },
        { 15, 7, 13, 5 }
    };

    // following r parameters have been found using standard deviation
    // that gives a decent result
    int r1 = r + roundf(92.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    int g1 = g + roundf(85.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    int b1 = b + roundf(65.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));

    return nearest_palette_index(r1, g1, b1, palette, palette_size);
}

uint8_t epaper_dither_4gray(int x, int y, uint8_t r, uint8_t g, uint8_t b,
    const uint8_t palette[][3], int palette_size)
{
    uint8_t exact_index;
    if (exact_palette_index(r, g, b, palette, palette_size, &exact_index)) {
        return exact_index;
    }

    const uint8_t m[4][4] = {
        { 0, 8, 2, 10 },
        { 12, 4, 14, 6 },
        { 3, 11, 1, 9 },
        { 15, 7, 13, 5 }
    };

    int yval = ((r << 1) + r + (g << 2) + b) >> 3;
    yval = clamp_u8(yval + ((2 * m[x % 4][y % 4]) - 15) * 3);

    return 3 - ((yval * 3 + 127) / 255);
}
