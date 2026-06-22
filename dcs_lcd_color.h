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

#ifndef _DCS_LCD_COLOR_H_
#define _DCS_LCD_COLOR_H_

#include <stdint.h>

#include <driver/spi_master.h>

// This functions is taken from:
// https://stackoverflow.com/questions/18937701/combining-two-16-bits-rgb-colors-with-alpha-blending
static inline uint16_t alpha_blend_rgb565(uint32_t fg, uint32_t bg, uint8_t alpha)
{
    alpha = (alpha + 4) >> 3;
    bg = (bg | (bg << 16)) & 0b00000111111000001111100000011111;
    fg = (fg | (fg << 16)) & 0b00000111111000001111100000011111;
    uint32_t result = ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
    return (uint16_t) ((result >> 16) | result);
}

static inline uint8_t rgba8888_get_alpha(uint32_t color)
{
    return color & 0xFF;
}

static inline uint16_t rgba8888_color_to_rgb565(uint32_t color)
{
    uint8_t r = color >> 24;
    uint8_t g = color >> 16;
    uint8_t b = color >> 8;

    return (((uint16_t) (r >> 3)) << 11) | (((uint16_t) (g >> 2)) << 5) | ((uint16_t) b >> 3);
}

static inline uint16_t display_color_to_rgb565(uint32_t color)
{
    return rgba8888_color_to_rgb565(color);
}

static inline uint16_t rgb565_color_to_surface(uint16_t color16)
{
    return (uint16_t) SPI_SWAP_DATA_TX(color16, 16);
}

static inline uint16_t display_color_to_surface(uint32_t color)
{
    uint16_t color16 = display_color_to_rgb565(color);

    return rgb565_color_to_surface(color16);
}

static inline uint16_t uint32_color_to_surface(uint32_t color)
{
    uint16_t color16 = rgba8888_color_to_rgb565(color);

    return rgb565_color_to_surface(color16);
}

#endif
