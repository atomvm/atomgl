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

#include "dcs_lcd_draw.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <utils.h>

#include "dcs_lcd_color.h"
#include "font_data.h"

int dcs_lcd_draw_image_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = rgba8888_color_to_rgb565(item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * width + (xpos - x);
    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        if (alpha == 0xFF) {
            uint16_t color = uint32_color_to_surface(img_pixel);
            pixmem16[drawn_pixels] = color;
        } else if (visible_bg) {
            uint16_t color = rgba8888_color_to_rgb565(img_pixel);
            uint16_t blended = alpha_blend_rgb565(color, bgcolor, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(blended);
        } else if (drawn_pixels > 0) {
            return drawn_pixels;
        } else {
            int run = 1;
            while (j + run < width && rgba8888_get_alpha(READ_32_UNALIGNED(pixels + run)) != 0xFF) {
                run++;
            }
            return -run;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

int dcs_lcd_draw_rect_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int width = item->width;
    uint16_t color = uint32_color_to_surface(item->brcolor);

    int drawn_pixels = 0;

    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        pixmem16[drawn_pixels] = color;
        drawn_pixels++;
    }

    return drawn_pixels;
}

int dcs_lcd_draw_shape_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item,
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
    return dcs_lcd_draw_rect_x(screen, xpos, ypos, run, item);
}

int dcs_lcd_draw_text_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;
    uint16_t fgcolor = uint32_color_to_surface(item->data.text_data.fgcolor);
    uint16_t bgcolor;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = uint32_color_to_surface(item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    char *text = (char *) item->data.text_data.text;

    int width = item->width;

    int drawn_pixels = 0;

    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

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
            pixmem16[drawn_pixels] = fgcolor;
        } else if (visible_bg) {
            pixmem16[drawn_pixels] = bgcolor;
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

int dcs_lcd_draw_scaled_cropped_img_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = rgba8888_color_to_rgb565(item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;

    int drawn_pixels = 0;

    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    struct ScaledCroppedRow src;
    display_items_scaled_cropped_row_init(&src, item, ypos - y);

    int j = xpos - x;
    while (j < width) {
        int run;
        uint32_t img_pixel = READ_32_UNALIGNED(display_items_scaled_cropped_row_run(&src, j, width, &run));
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        uint16_t color;
        if (alpha == 0xFF) {
            color = uint32_color_to_surface(img_pixel);
        } else if (visible_bg) {
            color = rgb565_color_to_surface(alpha_blend_rgb565(rgba8888_color_to_rgb565(img_pixel), bgcolor, alpha));
        } else if (drawn_pixels > 0) {
            return drawn_pixels;
        } else {
            int transparent = run;
            j += run;
            while (j < width
                && rgba8888_get_alpha(READ_32_UNALIGNED(display_items_scaled_cropped_row_run(&src, j, width, &run))) != 0xFF) {
                transparent += run;
                j += run;
            }
            return -transparent;
        }
        for (int k = 0; k < run; k++) {
            pixmem16[drawn_pixels++] = color;
        }
        j += run;
    }

    return drawn_pixels;
}

int dcs_lcd_draw_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, BaseDisplayItem *row)
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
                drawn_pixels = dcs_lcd_draw_image_x(screen, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveRect:
                drawn_pixels = dcs_lcd_draw_rect_x(screen, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveScaledCroppedImage:
                drawn_pixels = dcs_lcd_draw_scaled_cropped_img_x(screen, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveText:
                drawn_pixels = dcs_lcd_draw_text_x(screen, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveShape:
                drawn_pixels = dcs_lcd_draw_shape_x(screen, xpos, ypos, max_line_len, item, &run);
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
