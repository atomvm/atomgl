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

#include "mono_draw.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <utils.h>

#include "display_items.h"
#include "font_data.h"

static int get_color(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    // dither

    /*
     * Original bayer matrix
     *   { 0, 8, 2, 10 },
     *   { 12, 4, 14, 6 },
     *   { 3, 11, 1, 9 },
     *   { 15, 7, 13, 5 }
     *
     *   The following is calculated applying the following code element by element
     *   r = 255 / values / 4
     *   roundf(63.75 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
     */
    const int m[4][4] = {
        { -32, 0, -24, 8 },
        { 16, -16, 24, -8 },
        { -20, 12, -28, 4 },
        { 28, -4, 20, -12 }
    };

    int v = m[x % 4][y % 4];
    int out_r = r + v;
    int out_g = g + v;
    int out_b = b + v;
    // end of dither

    // get closest
    // float yval = 0.2126 * out_r + 0.7152 * out_g + 0.0722 * out_b;
    // the following is a fast formula
    int yval = ((out_r << 1) + out_r + (out_g << 2) + out_b) >> 3;

    return yval >= 128;
}

void mono_draw_pixel_x(const struct MonoScreen *screen,
    uint8_t *line_buf, int xpos, int color)
{
    if (xpos >= screen->w) {
        fprintf(stderr, "display buffer overflow: %i!\n", xpos);
        return;
    }

    int bpos = (xpos % 8);
    line_buf[xpos / 8] = (line_buf[xpos / 8] & ~(0x1 << bpos)) | (color << bpos);
}

int mono_draw_image_x(const struct MonoScreen *screen,
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
        if (rgba8888_get_alpha(img_pixel) != 0) {
            uint8_t r = img_pixel >> 24;
            uint8_t g = (img_pixel >> 16) & 0xFF;
            uint8_t b = (img_pixel >> 8) & 0xFF;

            int c = get_color(xpos + drawn_pixels, ypos, r, g, b);
            mono_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            int c = get_color(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
            mono_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        } else if (drawn_pixels > 0) {
            return drawn_pixels;
        } else {
            int run = 1;
            while (j + run < width && rgba8888_get_alpha(READ_32_UNALIGNED(pixels + run)) == 0) {
                run++;
            }
            return -run;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

int mono_draw_rect_x(const struct MonoScreen *screen,
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
        int c = get_color(xpos + drawn_pixels, ypos, r, g, b);
        mono_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        drawn_pixels++;
    }

    return drawn_pixels;
}

int mono_draw_shape_x(const struct MonoScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item,
    int *outside_run)
{
    if (display_items_shape_outside_run(item, xpos, ypos, outside_run)) {
        return 0;
    }
    bool inside;
    int run = shape_run(item->data.shape_data.shape, xpos, ypos, &inside);
    if (!inside) {
        display_items_shape_remember_outside(item, xpos, ypos, run);
        *outside_run = run;
        return 0;
    }
    if (run > max_line_len) {
        run = max_line_len;
    }
    return mono_draw_rect_x(screen, line_buf, xpos, ypos, run, item);
}

int mono_draw_text_x(const struct MonoScreen *screen,
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
            int c = get_color(xpos + drawn_pixels, ypos, fgcolor_r, fgcolor_g, fgcolor_b);
            mono_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            int c = get_color(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
            mono_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);

        } else if (drawn_pixels > 0) {
            return drawn_pixels;
        } else {
            int run = 1;
            while (j + run < width && (fontdata[((unsigned char) text[(j + run) / CHAR_WIDTH]) * 16 + ypos - y] & (1 << (7 - (j + run) % CHAR_WIDTH))) == 0) {
                run++;
            }
            return -run;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

int mono_draw_scaled_cropped_img_x(const struct MonoScreen *screen,
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
        int run;
        uint32_t img_pixel = READ_32_UNALIGNED(display_items_scaled_cropped_row_run(&src, j, width, &run));
        if (rgba8888_get_alpha(img_pixel) != 0) {
            uint8_t r = img_pixel >> 24;
            uint8_t g = (img_pixel >> 16) & 0xFF;
            uint8_t b = (img_pixel >> 8) & 0xFF;
            for (int k = 0; k < run; k++) {
                int c = get_color(xpos + drawn_pixels, ypos, r, g, b);
                mono_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);
                drawn_pixels++;
            }
        } else if (visible_bg) {
            for (int k = 0; k < run; k++) {
                int c = get_color(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
                mono_draw_pixel_x(screen, line_buf, xpos + drawn_pixels, c);
                drawn_pixels++;
            }
        } else if (drawn_pixels > 0) {
            return drawn_pixels;
        } else {
            int transparent = run;
            j += run;
            while (j < width
                && rgba8888_get_alpha(READ_32_UNALIGNED(display_items_scaled_cropped_row_run(&src, j, width, &run))) == 0) {
                transparent += run;
                j += run;
            }
            return -transparent;
        }
        j += run;
    }

    return drawn_pixels;
}

int mono_draw_x(const struct MonoScreen *screen,
    uint8_t *line_buf, int xpos, int ypos,
    BaseDisplayItem *row)
{
    int line_len = screen->w - xpos;
    int transparent_run = INT_MAX;

    for (BaseDisplayItem *item = row; item != NULL; item = item->next) {
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

        int max_line_len = (line_len < transparent_run) ? line_len : transparent_run;

        int run = 1;
        int drawn_pixels = 0;
        switch (item->primitive) {
            case PrimitiveImage:
                //fprintf(stderr, "Image\n");
                drawn_pixels = mono_draw_image_x(screen, line_buf, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveScaledCroppedImage:
                //fprintf(stderr, "ScaledCroppedImage\n");
                drawn_pixels = mono_draw_scaled_cropped_img_x(screen, line_buf, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveRect:
                //fprintf(stderr, "Rect\n");
                drawn_pixels = mono_draw_rect_x(screen, line_buf, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveText:
                //fprintf(stderr, "Text\n");
                drawn_pixels = mono_draw_text_x(screen, line_buf, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveShape:
                drawn_pixels = mono_draw_shape_x(screen, line_buf, xpos, ypos, max_line_len, item, &run);
                break;

            default: {
                fprintf(stderr, "unexpected display list command.\n");
            }
        }

        if (drawn_pixels > 0) {
            return drawn_pixels;
        }

        // Transparent for run pixels: images and text return it negated, shapes set it
        if (drawn_pixels < 0) {
            run = -drawn_pixels;
        }
        if (run < transparent_run) {
            transparent_run = run;
        }
    }

    return 1;
}
