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

#include <context.h>
#include <stdint.h>

// TODO: deprecated helper, remove this
static inline term context_make_atom(Context *ctx, AtomString string)
{
    return globalcontext_make_atom(ctx->global, string);
}

enum primitive
{
    Invalid = 0,
    Image,
    ScaledCroppedImage,
    Rect,
    Text
};

struct TextData
{
    uint32_t fgcolor;
    const char *text;
};

struct ImageData
{
    const char *pix;
};

struct ImageDataWithSize
{
    int width;
    int height;
    const char *pix;
};

struct BaseDisplayItem
{
    enum primitive primitive;
    int x;
    int y;
    int width;
    int height;
    uint32_t brcolor;
    union
    {
        struct ImageData image_data;
        struct ImageDataWithSize image_data_with_size;
        struct TextData text_data;
    } data;

    //used just for scaled cropped image
    int source_x;
    int source_y;
    int x_scale;
    int y_scale;
};

typedef struct BaseDisplayItem BaseDisplayItem;

static void init_item(BaseDisplayItem *item, term req, Context *ctx)
{
    term cmd = term_get_tuple_element(req, 0);

    if (cmd == context_make_atom(ctx, "\x5"
                                      "image")) {
        item->primitive = Image;
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
        item->primitive = ScaledCroppedImage;
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
        item->primitive = Rect;
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
            item->primitive = Text;
            item->height = 16;
            item->width = strlen(text) * 8;
            item->brcolor = brcolor;
            item->data.text_data.fgcolor = fgcolor;
            item->data.text_data.text = text;

        } else {
#ifdef ENABLE_UFONT
            AtomString handle_atom = globalcontext_atomstring_from_term(ctx->global, font);
            char handle[255];
            atom_string_to_c(handle_atom, handle, sizeof(handle));
            EpdFont *loaded_font = ufont_manager_find_by_handle(ufont_manager, handle);

            if (!loaded_font) {
                fprintf(stderr, "unsupported font: ");
                term_display(stderr, font, ctx);
                fprintf(stderr, "\n");
                return;
            }

            EpdFontProperties props = epd_font_properties_default();
            EpdRect rect = epd_get_string_rect(loaded_font, text, 0, 0, 0, &props);

            struct Surface surface;
            surface.width = rect.width;
            surface.height = rect.height;
            surface.buffer = malloc(rect.width * rect.height * BPP);
            memset(surface.buffer, 0, rect.width * rect.height * BPP);
            int text_x = 0;
            int text_y = loaded_font->ascender;
            enum EpdDrawError res = epd_write_default(loaded_font, text, &text_x, &text_y, &surface);
            free(text);
            if (res != EPD_DRAW_SUCCESS) {
                fprintf(stderr, "Failed to draw text. Error code: %i\n", res);
                return;
            }

            item->primitive = Image;
            item->width = surface.width;
            item->height = surface.height;
            item->brcolor = 0;
            //FIXME: surface buffer leak
            item->data.image_data.pix = surface.buffer;
#else
            fprintf(stderr, "unsupported font: ");
            term_display(stderr, font, ctx);
            fprintf(stderr, "\n");
            item->primitive = Text;
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

        item->primitive = Invalid;
        item->x = -1;
        item->y = -1;
        item->width = 1;
        item->height = 1;
    }
}

static void destroy_items(BaseDisplayItem *items, int items_count)
{
    for (int i = 0; i < items_count; i++) {
        BaseDisplayItem *item = &items[i];

        switch (item->primitive) {
            case Image:
                break;

            case Rect:
                break;

            case Text:
                free((char *) item->data.text_data.text);
                break;

            default: {
                break;
            }
        }
    }

    free(items);
}
