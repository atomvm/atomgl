/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2022 Davide Bettio <davide@uninstall.it>
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

#include "display_items.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <interop.h>

#ifdef ENABLE_UFONT
#include "ufontlib.h"
extern UFontManager *ufont_manager;

struct Surface
{
    int width;
    int height;
    void *buffer;
    uint32_t fg_color; // 0xRRGGBBAA from Erlang color << 8 | 0xFF
};

#define BPP 4

void epd_draw_pixel(int xpos, int ypos, uint8_t color, void *buffer)
{
    struct Surface *surface = buffer;

    if (xpos < 0 || ypos < 0 || xpos >= surface->width
            || ypos >= surface->height) {
        return;
    }

    uint8_t *pixel = ((uint8_t *) surface->buffer)
            + (surface->width * ypos + xpos) * sizeof(uint32_t);

    // The `color` parameter is the LUT-mapped glyph value from
    // draw_char: 0 = full foreground (fg_color=0 in default props),
    // 240 = full background (bg_color=15), in steps of 16. Render
    // the foreground RGB on transparent with anti-aliased alpha
    // derived from the inverted grayscale.
    uint8_t alpha = (15 - (color >> 4)) * 17;
    pixel[0] = (surface->fg_color >> 24) & 0xFFu;
    pixel[1] = (surface->fg_color >> 16) & 0xFFu;
    pixel[2] = (surface->fg_color >> 8) & 0xFFu;
    pixel[3] = alpha;
}
#endif /* ENABLE_UFONT */

static bool parse_image_tuple(term img, Context *ctx, int *width, int *height, const char **pix, bool *rgb565_pixels)
{
    term format = term_get_tuple_element(img, 0);
    *width = term_to_int(term_get_tuple_element(img, 1));
    *height = term_to_int(term_get_tuple_element(img, 2));
    term data_term = term_get_tuple_element(img, 3);

    if (*width <= 0 || *height <= 0) {
        fprintf(stderr, "invalid image dimensions: %ix%i\n", *width, *height);
        return false;
    }

    size_t bytes_per_pixel;
    if (format == context_make_atom(ctx, "\x8"
                                         "rgba8888")) {
        *rgb565_pixels = false;
        bytes_per_pixel = 4;
    } else if (format == context_make_atom(ctx, "\x6"
                                                "rgb565")) {
        *rgb565_pixels = true;
        bytes_per_pixel = 2;
    } else {
        fprintf(stderr, "unsupported image format: ");
        term_display(stderr, format, ctx);
        fprintf(stderr, "\n");
        return false;
    }

    size_t expected = (size_t) *width * (size_t) *height * bytes_per_pixel;
    if (term_binary_size(data_term) < expected) {
        fprintf(stderr, "image binary too small (%lu < %zu)\n",
            (unsigned long) term_binary_size(data_term), expected);
        return false;
    }

    *pix = term_binary_data(data_term);
    return true;
}

void display_items_init_item(BaseDisplayItem *item, term req, Context *ctx)
{
    memset(item, 0, sizeof(*item));

    term cmd = term_get_tuple_element(req, 0);

    if (cmd == context_make_atom(ctx, "\x5"
                                      "image")) {
        item->primitive = PrimitiveImage;
        item->x = term_to_int(term_get_tuple_element(req, 1));
        item->y = term_to_int(term_get_tuple_element(req, 2));

        term bgcolor = term_get_tuple_element(req, 3);
        if (bgcolor == context_make_atom(ctx, "\xB"
                                              "transparent")) {
            item->brcolor = 0;
        } else {
            item->brcolor = ((uint32_t) term_to_int(bgcolor)) << 8 | 0xFF;
        }

        term img = term_get_tuple_element(req, 4);

        int width;
        int height;
        const char *pix;
        if (!parse_image_tuple(img, ctx, &width, &height, &pix, &item->rgb565_pixels)) {
            return;
        }
        item->width = width;
        item->height = height;
        item->data.image_data.pix = pix;

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x14", "scaled_cropped_image"))) {
        item->primitive = PrimitiveScaledCroppedImage;
        item->x = term_to_int(term_get_tuple_element(req, 1));
        item->y = term_to_int(term_get_tuple_element(req, 2));
        item->width = term_to_int(term_get_tuple_element(req, 3));
        item->height = term_to_int(term_get_tuple_element(req, 4));

        term bgcolor = term_get_tuple_element(req, 5);
        if (bgcolor == globalcontext_make_atom(ctx->global, "\xB"
                                              "transparent")) {
            item->brcolor = 0;
        } else {
            item->brcolor = ((uint32_t) term_to_int(bgcolor)) << 8 | 0xFF;
        }

        item->source_x = term_to_int(term_get_tuple_element(req, 6));
        item->source_y = term_to_int(term_get_tuple_element(req, 7));
        item->x_scale = term_to_int(term_get_tuple_element(req, 8));
        item->y_scale = term_to_int(term_get_tuple_element(req, 9));

        if (item->x_scale <= 0 || item->y_scale <= 0) {
            fprintf(stderr, "scaled_cropped_image: scale factors must be > 0\n");
            return;
        }
        if (item->source_x < 0 || item->source_y < 0) {
            fprintf(stderr, "scaled_cropped_image: source offsets must be >= 0\n");
            return;
        }

        // 10th element is for opts, but right now no opts are supported

        term img = term_get_tuple_element(req, 11);

        int img_width;
        int img_height;
        const char *pix;
        if (!parse_image_tuple(img, ctx, &img_width, &img_height, &pix, &item->rgb565_pixels)) {
            return;
        }
        item->data.image_data_with_size.width = img_width;
        item->data.image_data_with_size.height = img_height;
        item->data.image_data_with_size.pix = pix;

        if (item->source_x >= item->data.image_data_with_size.width
                || item->source_y >= item->data.image_data_with_size.height) {
            fprintf(stderr, "scaled_cropped_image: source offset outside image\n");
            return;
        }

    } else if (cmd == context_make_atom(ctx, "\x4"
                                             "rect")) {
        item->primitive = PrimitiveRect;
        item->x = term_to_int(term_get_tuple_element(req, 1));
        item->y = term_to_int(term_get_tuple_element(req, 2));
        item->width = term_to_int(term_get_tuple_element(req, 3));
        item->height = term_to_int(term_get_tuple_element(req, 4));
        item->brcolor = term_to_int(term_get_tuple_element(req, 5)) << 8 | 0xFF;

    } else if (cmd == context_make_atom(ctx, "\x4"
                                             "text")) {
        item->x = term_to_int(term_get_tuple_element(req, 1));
        item->y = term_to_int(term_get_tuple_element(req, 2));
        uint32_t fgcolor = term_to_int(term_get_tuple_element(req, 4)) << 8 | 0xFF;
        uint32_t brcolor;
        term bgcolor = term_get_tuple_element(req, 5);
        if (bgcolor == globalcontext_make_atom(ctx->global, "\xB"
                                              "transparent")) {
            brcolor = 0;
        } else {
            brcolor = ((uint32_t) term_to_int(bgcolor)) << 8 | 0xFF;
        }
        term text_term = term_get_tuple_element(req, 6);
        int ok;
        char *text = interop_term_to_string(text_term, &ok);
        if (!ok) {
            fprintf(stderr, "invalid text.\n");
            return;
        }

        term font = term_get_tuple_element(req, 3);

        if (font == globalcontext_make_atom(ctx->global, "\xB" "default16px")) {
            item->primitive = PrimitiveText;
            item->height = 16;
            item->width = strlen(text) * 8;
            item->brcolor = brcolor;
            item->data.text_data.fgcolor = fgcolor;
            item->data.text_data.text = text;

        } else {
#ifdef ENABLE_UFONT
            char *handle = interop_atom_to_string(ctx, font);
            EpdFont *loaded_font = NULL;
            if (handle != NULL) {
                loaded_font = ufont_manager_find_by_handle(ufont_manager, handle);
                free(handle);
            }

            if (!loaded_font) {
                fprintf(stderr, "unsupported font: ");
                term_display(stderr, font, ctx);
                fprintf(stderr, "\n");
                free(text);
                return;
            }

            EpdFontProperties props = epd_font_properties_default();
            EpdRect rect = epd_get_string_rect(loaded_font, text, 0, 0, 0, &props);

            struct Surface surface;
            surface.width = rect.width;
            surface.height = rect.height;
            if (rect.width <= 0 || rect.height <= 0) {
                fprintf(stderr, "invalid ufont surface size (%ix%i)\n",
                    rect.width, rect.height);
                free(text);
                return;
            }
            size_t pixel_count = (size_t) rect.width * (size_t) rect.height;
            if (pixel_count > SIZE_MAX / BPP) {
                fprintf(stderr, "ufont surface size overflow (%ix%i)\n",
                    rect.width, rect.height);
                free(text);
                return;
            }
            size_t surface_bytes = pixel_count * BPP;
            surface.buffer = malloc(surface_bytes);
            if (!surface.buffer) {
                fprintf(stderr, "Failed to allocate ufont surface (%ix%i)\n",
                    rect.width, rect.height);
                free(text);
                return;
            }
            memset(surface.buffer, 0, surface_bytes);
            surface.fg_color = fgcolor;
            int text_x = 0;
            int text_y = loaded_font->ascender;
            enum EpdDrawError res = epd_write_default(loaded_font, text, &text_x, &text_y, &surface);
            free(text);
            if (res != EPD_DRAW_SUCCESS) {
                fprintf(stderr, "Failed to draw text. Error code: %i\n", res);
                free(surface.buffer);
                return;
            }

            item->primitive = PrimitiveImage;
            item->width = surface.width;
            item->height = surface.height;
            item->brcolor = brcolor;
            item->data.image_data.pix = surface.buffer;
            item->owns_data = true;
#else
            fprintf(stderr, "unsupported font: ");
            term_display(stderr, font, ctx);
            fprintf(stderr, "\n");
            item->primitive = PrimitiveText;
            item->height = 16;
            item->width = strlen(text) * 8;
            item->brcolor = brcolor;
            item->data.text_data.fgcolor = fgcolor;
            item->data.text_data.text = text;

#endif
        }

    } else {
        fprintf(stderr, "unexpected display list command: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
    }
}

void display_items_delete(BaseDisplayItem items[], size_t items_len)
{
    for (size_t i = 0; i < items_len; i++) {
        BaseDisplayItem *item = &items[i];

        switch (item->primitive) {
            case PrimitiveImage:
                if (item->owns_data) {
                    free((void *) item->data.image_data.pix);
                }
                break;

            case PrimitiveRect:
                break;

            case PrimitiveText:
                free((char *) item->data.text_data.text);
                break;

            default: {
                break;
            }
        }
    }

    free(items);
}
