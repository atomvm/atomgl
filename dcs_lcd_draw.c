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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <utils.h>

#include "dcs_lcd_color.h"
#include "font_data.h"

int dcs_lcd_find_max_line_len(const struct DCSLCDScreen *screen,
    BaseDisplayItem items[], size_t items_len, int xpos, int ypos)
{
    int line_len = screen->w - xpos;

    for (size_t i = 0; i < items_len; i++) {
        BaseDisplayItem *item = &items[i];

        if ((xpos < item->x) && (ypos >= item->y) && (ypos < item->y + item->height)) {
            int len_to_item = item->x - xpos;
            line_len = (line_len > len_to_item) ? len_to_item : line_len;
        }
    }

    return line_len;
}

static bool dcs_lcd_resolve_pixel_rgb565(const struct DCSLCDScreen *screen,
    int xpos, int ypos, BaseDisplayItem items[], size_t items_len, size_t start_index, uint16_t *out_color);

static bool dcs_lcd_image_pixel_rgb565(const struct DCSLCDScreen *screen,
    BaseDisplayItem *item, int xpos, int ypos, BaseDisplayItem items[], size_t items_len, size_t item_index, uint16_t *out_color)
{
    int x = item->x;
    int y = item->y;
    int rel_x = xpos - x;
    int rel_y = ypos - y;

    if (item->rgb565_pixels) {
        const uint16_t *pixels = ((const uint16_t *) item->data.image_data.pix)
            + (rel_y * item->width) + rel_x;
        *out_color = pixels[0];
        return true;
    }

    uint32_t *pixels = ((uint32_t *) item->data.image_data.pix) + (rel_y * item->width) + rel_x;
    uint32_t img_pixel = READ_32_UNALIGNED(pixels);
    uint8_t alpha = rgba8888_get_alpha(img_pixel);

    if (alpha == 0xFF) {
        *out_color = rgba8888_color_to_rgb565(img_pixel);
        return true;
    }
    if (item->brcolor != 0) {
        uint16_t color = rgba8888_color_to_rgb565(img_pixel);
        uint16_t bgcolor = display_color_to_rgb565(item->brcolor);
        *out_color = alpha_blend_rgb565(color, bgcolor, alpha);
        return true;
    }
    if (alpha > 0) {
        uint16_t lower = 0;
        uint16_t color = rgba8888_color_to_rgb565(img_pixel);
        (void) dcs_lcd_resolve_pixel_rgb565(screen, xpos, ypos, items, items_len, item_index + 1, &lower);
        *out_color = alpha_blend_rgb565(color, lower, alpha);
        return true;
    }

    return dcs_lcd_resolve_pixel_rgb565(screen, xpos, ypos, items, items_len, item_index + 1, out_color);
}

static bool dcs_lcd_scaled_image_pixel_rgb565(const struct DCSLCDScreen *screen,
    BaseDisplayItem *item, int xpos, int ypos, BaseDisplayItem items[], size_t items_len, size_t item_index, uint16_t *out_color)
{
    int x = item->x;
    int y = item->y;
    int img_width = item->data.image_data_with_size.width;
    int img_height = item->data.image_data_with_size.height;

    if (item->x_scale <= 0 || item->y_scale <= 0 || item->source_x < 0 || item->source_y < 0
            || item->source_x >= img_width || item->source_y >= img_height) {
        return false;
    }

    int source_x = item->source_x + ((xpos - x) / item->x_scale);
    int source_y = item->source_y + ((ypos - y) / item->y_scale);
    if (source_x < 0 || source_y < 0 || source_x >= img_width || source_y >= img_height) {
        return false;
    }

    if (item->rgb565_pixels) {
        const uint16_t *pixels16 = ((const uint16_t *) item->data.image_data_with_size.pix)
            + (source_y * img_width) + source_x;
        *out_color = pixels16[0];
        return true;
    }

    uint32_t *pixels = ((uint32_t *) item->data.image_data_with_size.pix) + (source_y * img_width) + source_x;
    uint32_t img_pixel = READ_32_UNALIGNED(pixels);
    uint8_t alpha = rgba8888_get_alpha(img_pixel);

    if (alpha == 0xFF) {
        *out_color = rgba8888_color_to_rgb565(img_pixel);
        return true;
    }
    if (item->brcolor != 0) {
        uint16_t color = rgba8888_color_to_rgb565(img_pixel);
        uint16_t bgcolor = display_color_to_rgb565(item->brcolor);
        *out_color = alpha_blend_rgb565(color, bgcolor, alpha);
        return true;
    }
    if (alpha > 0) {
        uint16_t lower = 0;
        uint16_t color = rgba8888_color_to_rgb565(img_pixel);
        (void) dcs_lcd_resolve_pixel_rgb565(screen, xpos, ypos, items, items_len, item_index + 1, &lower);
        *out_color = alpha_blend_rgb565(color, lower, alpha);
        return true;
    }

    return dcs_lcd_resolve_pixel_rgb565(screen, xpos, ypos, items, items_len, item_index + 1, out_color);
}

static bool dcs_lcd_text_pixel_rgb565(const struct DCSLCDScreen *screen,
    BaseDisplayItem *item, int xpos, int ypos, BaseDisplayItem items[], size_t items_len, size_t item_index, uint16_t *out_color)
{
    int x = item->x;
    int y = item->y;
    char *text = (char *) item->data.text_data.text;
    int char_index = (xpos - x) / CHAR_WIDTH;
    char c = text[char_index];
    unsigned const char *glyph = fontdata + ((unsigned char) c) * 16;
    unsigned char row = glyph[ypos - y];
    int k = (xpos - x) % CHAR_WIDTH;

    if (row & (1 << (7 - k))) {
        *out_color = display_color_to_rgb565(item->data.text_data.fgcolor);
        return true;
    }
    if (item->brcolor != 0) {
        *out_color = display_color_to_rgb565(item->brcolor);
        return true;
    }

    return dcs_lcd_resolve_pixel_rgb565(screen, xpos, ypos, items, items_len, item_index + 1, out_color);
}

static bool dcs_lcd_resolve_pixel_rgb565(const struct DCSLCDScreen *screen,
    int xpos, int ypos, BaseDisplayItem items[], size_t items_len, size_t start_index, uint16_t *out_color)
{
    for (size_t i = start_index; i < items_len; i++) {
        BaseDisplayItem *item = &items[i];
        if ((xpos < item->x) || (xpos >= item->x + item->width) || (ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }

        switch (item->primitive) {
            case PrimitiveImage:
                if (dcs_lcd_image_pixel_rgb565(screen, item, xpos, ypos, items, items_len, i, out_color)) {
                    return true;
                }
                break;
            case PrimitiveRect:
                *out_color = display_color_to_rgb565(item->brcolor);
                return true;
            case PrimitiveScaledCroppedImage:
                if (dcs_lcd_scaled_image_pixel_rgb565(screen, item, xpos, ypos, items, items_len, i, out_color)) {
                    return true;
                }
                break;
            case PrimitiveText:
                if (dcs_lcd_text_pixel_rgb565(screen, item, xpos, ypos, items, items_len, i, out_color)) {
                    return true;
                }
                break;
            default:
                break;
        }
    }
    return false;
}

int dcs_lcd_draw_image_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item,
    BaseDisplayItem items[], size_t items_len, size_t item_index)
{
    int x = item->x;
    int y = item->y;

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    if (item->rgb565_pixels) {
        const uint16_t *pixels16 = ((const uint16_t *) data) + (ypos - y) * item->width + (xpos - x);
        for (int j = xpos - x; j < width; j++) {
            pixmem16[drawn_pixels] = rgb565_color_to_surface(pixels16[j]);
            drawn_pixels++;
        }
        return drawn_pixels;
    }

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = display_color_to_rgb565(item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * item->width + (xpos - x);

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
        } else if (alpha > 0) {
            uint16_t color = rgba8888_color_to_rgb565(img_pixel);
            uint16_t lower = rgb565_color_to_surface(pixmem16[drawn_pixels]);
            uint16_t resolved = 0;
            if (dcs_lcd_resolve_pixel_rgb565(screen, xpos + drawn_pixels, ypos, items, items_len, item_index + 1, &resolved)) {
                lower = resolved;
            }
            uint16_t blended = alpha_blend_rgb565(color, lower, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(blended);
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
    uint16_t color = display_color_to_surface(item->brcolor);

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

int dcs_lcd_draw_text_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item,
    BaseDisplayItem items[], size_t items_len, size_t item_index)
{
    int x = item->x;
    int y = item->y;
    uint16_t fgcolor = display_color_to_surface(item->data.text_data.fgcolor);
    uint16_t bgcolor;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = display_color_to_surface(item->brcolor);
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
        } else {
            uint16_t lower = 0;
            if (!dcs_lcd_resolve_pixel_rgb565(screen, xpos + drawn_pixels, ypos, items, items_len, item_index + 1, &lower)) {
                return drawn_pixels;
            }
            pixmem16[drawn_pixels] = rgb565_color_to_surface(lower);
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

int dcs_lcd_draw_scaled_cropped_img_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item,
    BaseDisplayItem items[], size_t items_len, size_t item_index)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = display_color_to_rgb565(item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data_with_size.pix;

    int drawn_pixels = 0;

    int y_scale = item->y_scale;
    int x_scale = item->x_scale;
    int img_width = item->data.image_data_with_size.width;
    int img_height = item->data.image_data_with_size.height;

    if (x_scale <= 0 || y_scale <= 0 || item->source_x < 0 || item->source_y < 0
            || item->source_x >= img_width || item->source_y >= img_height) {
        return 0;
    }

    if (item->source_x + (width / x_scale) > img_width) {
        width = (img_width - item->source_x) * x_scale;
    }

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    if (width <= 0) {
        return 0;
    }

    int rel_y = ypos - y;
    int rel_x = xpos - x;
    int source_y = item->source_y + (rel_y / y_scale);
    int source_x = item->source_x + (rel_x / x_scale);
    if (source_y < 0 || source_y >= img_height || source_x < 0 || source_x >= img_width) {
        return 0;
    }

    uint32_t *pixels = ((uint32_t *) data) + (source_y * img_width) + source_x;
    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (item->rgb565_pixels) {
        const uint16_t *pixels16 = (const uint16_t *) data;
        for (int j = rel_x; j < width; j++) {
            int sample_x = item->source_x + (j / x_scale);
            int sample_y = item->source_y + (rel_y / y_scale);
            if (sample_x < 0 || sample_x >= img_width || sample_y < 0 || sample_y >= img_height) {
                break;
            }
            const uint16_t *src = pixels16 + (sample_y * img_width) + sample_x;
            pixmem16[drawn_pixels] = rgb565_color_to_surface(src[0]);
            drawn_pixels++;
        }
        return drawn_pixels;
    }

    for (int j = rel_x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        if (alpha == 0xFF) {
            uint16_t color = uint32_color_to_surface(img_pixel);
            pixmem16[drawn_pixels] = color;
        } else if (visible_bg) {
            uint16_t color = rgba8888_color_to_rgb565(img_pixel);
            uint16_t blended = alpha_blend_rgb565(color, bgcolor, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(blended);
        } else if (alpha > 0) {
            uint16_t color = rgba8888_color_to_rgb565(img_pixel);
            uint16_t lower = rgb565_color_to_surface(pixmem16[drawn_pixels]);
            uint16_t resolved = 0;
            if (dcs_lcd_resolve_pixel_rgb565(screen, xpos + drawn_pixels, ypos, items, items_len, item_index + 1, &resolved)) {
                lower = resolved;
            }
            uint16_t blended = alpha_blend_rgb565(color, lower, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(blended);
        }
        drawn_pixels++;
        int next_rel_x = j + 1;
        int next_source_x = item->source_x + (next_rel_x / x_scale);
        int next_source_y = item->source_y + (rel_y / y_scale);
        if (next_source_x < 0 || next_source_x >= img_width
                || next_source_y < 0 || next_source_y >= img_height) {
            break;
        }
        pixels = ((uint32_t *) data) + (next_source_y * img_width) + next_source_x;
    }

    return drawn_pixels;
}

int dcs_lcd_draw_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, BaseDisplayItem items[], size_t items_len)
{
    bool below = false;

    for (size_t i = 0; i < items_len; i++) {
        BaseDisplayItem *item = &items[i];
        if ((xpos < item->x) || (xpos >= item->x + item->width) || (ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }

        int max_line_len = below ? 1 : dcs_lcd_find_max_line_len(screen, items, i, xpos, ypos);

        int drawn_pixels = 0;
        switch (items[i].primitive) {
            case PrimitiveImage:
                drawn_pixels = dcs_lcd_draw_image_x(screen, xpos, ypos, max_line_len, item, items, items_len, i);
                break;

            case PrimitiveRect:
                drawn_pixels = dcs_lcd_draw_rect_x(screen, xpos, ypos, max_line_len, item);
                break;

            case PrimitiveScaledCroppedImage:
                drawn_pixels = dcs_lcd_draw_scaled_cropped_img_x(screen, xpos, ypos, max_line_len, item, items, items_len, i);
                break;

            case PrimitiveText:
                drawn_pixels = dcs_lcd_draw_text_x(screen, xpos, ypos, max_line_len, item, items, items_len, i);
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
