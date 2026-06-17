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

#include <interop.h>

#ifdef ENABLE_UFONT
#include "ufontlib.h"
extern UFontManager *ufont_manager;

struct Surface
{
    int width;
    int height;
    void *buffer;
    uint32_t fg_color; // RGBA8888 little-endian byte order with the
                       // alpha byte cleared; ORed with the per-pixel
                       // alpha in epd_draw_pixel.
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

        term format = term_get_tuple_element(img, 0);
        if (format != context_make_atom(ctx, "\x8"
                                             "rgba8888")) {
            fprintf(stderr, "unsupported image format: ");
            term_display(stderr, format, ctx);
            fprintf(stderr, "\n");
            return;
        }
        item->width = term_to_int(term_get_tuple_element(img, 1));
        item->height = term_to_int(term_get_tuple_element(img, 2));
        item->data.image_data.pix = term_binary_data(term_get_tuple_element(img, 3));

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

        // 10th element is for opts, but right now no opts are supported

        term img = term_get_tuple_element(req, 11);

        term format = term_get_tuple_element(img, 0);
        if (format != globalcontext_make_atom(ctx->global, "\x8"
                                             "rgba8888")) {
            fprintf(stderr, "unsupported image format: ");
            term_display(stderr, format, ctx);
            fprintf(stderr, "\n");
            return;
        }
        item->data.image_data_with_size.width = term_to_int(term_get_tuple_element(img, 1));
        item->data.image_data_with_size.height = term_to_int(term_get_tuple_element(img, 2));
        item->data.image_data_with_size.pix = term_binary_data(term_get_tuple_element(img, 3));

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
            surface.buffer = malloc(rect.width * rect.height * BPP);
            if (!surface.buffer) {
                fprintf(stderr, "Failed to allocate ufont surface (%ix%i)\n",
                    rect.width, rect.height);
                free(text);
                return;
            }
            memset(surface.buffer, 0, rect.width * rect.height * BPP);
            // Convert Erlang fgcolor (0xRRGGBBAA) to RGBA8888 little-
            // endian byte order (R in low byte, alpha byte cleared) so
            // epd_draw_pixel can OR it with the per-pixel alpha.
            surface.fg_color = ((fgcolor >> 24) & 0xFFu)
                    | (((fgcolor >> 16) & 0xFFu) << 8)
                    | (((fgcolor >> 8) & 0xFFu) << 16);
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
