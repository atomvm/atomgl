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

#include "epaper_draw.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <utils.h>

#include "display_items.h"
#include "epaper_color.h"
#include "epaper_screen.h"
#include "font_data.h"

void epaper_draw_pixel_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, uint8_t c)
{
    if (xpos >= screen->w) {
        fprintf(stderr, "buf ovf!\n");
        return;
    }

    if ((xpos & 1) == 0) {
        line_buf[xpos / 2] = (line_buf[xpos / 2] & 0xF) | (c << 4);
    } else {
        line_buf[xpos / 2] = (line_buf[xpos / 2] & 0xF0) | c;
    }
}

int epaper_draw_image_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    int bgcolor_r;
    int bgcolor_g;
    int bgcolor_b;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor_r = (item->brcolor >> 24) & 0xFF;
        bgcolor_g = (item->brcolor >> 16) & 0xFF;
        bgcolor_b = (item->brcolor >> 8) & 0xFF;
        visible_bg = true;
    } else {
        bgcolor_r = 0;
        bgcolor_g = 0;
        bgcolor_b = 0;
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * width + (xpos - x);

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        if ((*pixels >> 24) & 0xFF) {
            uint8_t r = img_pixel >> 24;
            uint8_t g = (img_pixel >> 16) & 0xFF;
            uint8_t b = (img_pixel >> 8) & 0xFF;

            uint8_t c = epaper_dither_acep7(xpos + drawn_pixels, ypos, r, g, b,
                screen->palette, screen->palette_size);
            epaper_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = epaper_dither_acep7(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b,
                screen->palette, screen->palette_size);
            epaper_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

int epaper_draw_rect_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item)
{
    int x = item->x;
    int width = item->width;

    uint8_t r = (item->brcolor >> 24) & 0xFF;
    uint8_t g = (item->brcolor >> 16) & 0xFF;
    uint8_t b = (item->brcolor >> 8) & 0xFF;

    int drawn_pixels = 0;

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint8_t c = epaper_dither_acep7(xpos + drawn_pixels, ypos, r, g, b,
            screen->palette, screen->palette_size);
        epaper_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);
        drawn_pixels++;
    }

    return drawn_pixels;
}

int epaper_draw_text_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;
    bool visible_bg;

    int fgcolor_r = (item->data.text_data.fgcolor >> 24) & 0xFF;
    int fgcolor_g = (item->data.text_data.fgcolor >> 16) & 0xFF;
    int fgcolor_b = (item->data.text_data.fgcolor >> 8) & 0xFF;

    int bgcolor_r;
    int bgcolor_g;
    int bgcolor_b;

    if (item->brcolor != 0) {
        bgcolor_r = (item->brcolor >> 24) & 0xFF;
        bgcolor_g = (item->brcolor >> 16) & 0xFF;
        bgcolor_b = (item->brcolor >> 8) & 0xFF;
        visible_bg = true;
    } else {
        bgcolor_r = 0;
        bgcolor_g = 0;
        bgcolor_b = 0;
        visible_bg = false;
    }

    char *text = (char *) item->data.text_data.text;

    int width = item->width;

    int drawn_pixels = 0;

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        int char_index = j / CHAR_WIDTH;
        char c = text[char_index];
        unsigned const char *glyph = fontdata + ((unsigned char) c) * 16;

        unsigned char row = glyph[ypos - y];

        bool opaque;
        int k = j % CHAR_WIDTH;
        if (row & (1 << (7 - k))) {
            opaque = true;
        } else {
            opaque = false;
        }

        if (opaque) {
            uint8_t c = epaper_dither_acep7(xpos + drawn_pixels, ypos, fgcolor_r, fgcolor_g, fgcolor_b,
                screen->palette, screen->palette_size);
            epaper_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = epaper_dither_acep7(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b,
                screen->palette, screen->palette_size);
            epaper_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

int epaper_draw_scaled_cropped_img_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    int bgcolor_r;
    int bgcolor_g;
    int bgcolor_b;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor_r = (item->brcolor >> 24) & 0xFF;
        bgcolor_g = (item->brcolor >> 16) & 0xFF;
        bgcolor_b = (item->brcolor >> 8) & 0xFF;
        visible_bg = true;
    } else {
        bgcolor_r = 0;
        bgcolor_g = 0;
        bgcolor_b = 0;
        visible_bg = false;
    }

    int width = item->width;

    int drawn_pixels = 0;

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    struct ScaledCroppedRow src;
    display_items_scaled_cropped_row_init(&src, item, ypos - y);

    int j = xpos - x;
    while (j < width) {
        int span_end = display_items_scaled_cropped_span(&src, j, width);
        for (; j < span_end; j++) {
            const uint32_t *pixels = display_items_scaled_cropped_row_pixel(&src, j);
            uint32_t img_pixel = READ_32_UNALIGNED(pixels);
            if ((*pixels >> 24) & 0xFF) {
                uint8_t r = img_pixel >> 24;
                uint8_t g = (img_pixel >> 16) & 0xFF;
                uint8_t b = (img_pixel >> 8) & 0xFF;

                uint8_t c = epaper_dither_acep7(xpos + drawn_pixels, ypos, r, g, b,
                    screen->palette, screen->palette_size);
                epaper_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

            } else if (visible_bg) {
                uint8_t c = epaper_dither_acep7(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b,
                    screen->palette, screen->palette_size);
                epaper_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

            } else {
                return drawn_pixels;
            }
            drawn_pixels++;
        }
    }

    return drawn_pixels;
}

int epaper_draw_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos,
    BaseDisplayItem items[], size_t items_len)
{
    int line_len = screen->w - xpos;
    bool below = false;

    for (size_t i = 0; i < items_len; i++) {
        BaseDisplayItem *item = &items[i];
        if ((ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }
        if (xpos < item->x) {
            int len_to_item = item->x - xpos;
            if (len_to_item < line_len) {
                line_len = len_to_item;
            }
            continue;
        }
        if (xpos >= item->x + item->width) {
            continue;
        }

        int max_line_len = below ? 1 : line_len;

        int drawn_pixels = 0;
        switch (items[i].primitive) {
            case PrimitiveImage:
                drawn_pixels = epaper_draw_image_x(screen, line_buf, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveScaledCroppedImage:
                drawn_pixels = epaper_draw_scaled_cropped_img_x(screen, line_buf, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveRect:
                drawn_pixels = epaper_draw_rect_x(screen, line_buf, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveText:
                drawn_pixels = epaper_draw_text_x(screen, line_buf, xpos, ypos, max_line_len, item);
                break;

            default: {
                fprintf(stderr, "unexpected display list command.\n");
            }
        }

        if (drawn_pixels != 0) {
            return drawn_pixels;
        }

        below = true;
    }

    return 1;
}
