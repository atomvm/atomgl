/*
 * This file is part of AtomVM.
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

enum primitive
{
    Invalid = 0,
    Image,
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
        struct TextData text_data;
    } data;
};

typedef struct BaseDisplayItem BaseDisplayItem;

static int draw_image_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item);
static int draw_rect_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item);
static int draw_text_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item);

static int find_max_line_len(BaseDisplayItem *items, int count, int xpos, int ypos)
{
    int line_len = DISPLAY_WIDTH - xpos;

    for (int i = 0; i < count; i++) {
        BaseDisplayItem *item = &items[i];

        if ((xpos < item->x) && (ypos >= item->y) && (ypos < item->y + item->height)) {
            int len_to_item = item->x - xpos;
            line_len = (line_len > len_to_item) ? len_to_item : line_len;
        }
    }

    return line_len;
}

static int draw_x(uint8_t *line_buf, int xpos, int ypos, BaseDisplayItem *items, int items_count)
{
    bool below = false;

    for (int i = 0; i < items_count; i++) {
        BaseDisplayItem *item = &items[i];
        if ((xpos < item->x) || (xpos >= item->x + item->width) || (ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }

        int max_line_len = below ? 1 : find_max_line_len(items, i, xpos, ypos);

        int drawn_pixels = 0;
        switch (items[i].primitive) {
            case Image:
                //fprintf(stderr, "Image\n");
                drawn_pixels = draw_image_x(line_buf, xpos, ypos, max_line_len, item);
                break;

            case Rect:
                //fprintf(stderr, "Rect\n");
                drawn_pixels = draw_rect_x(line_buf, xpos, ypos, max_line_len, item);
                break;

            case Text:
                //fprintf(stderr, "Text\n");
                drawn_pixels = draw_text_x(line_buf, xpos, ypos, max_line_len, item);
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
        item->primitive = Text;
        item->x = term_to_int(term_get_tuple_element(req, 1));
        item->y = term_to_int(term_get_tuple_element(req, 2));

        item->data.text_data.fgcolor = term_to_int(term_get_tuple_element(req, 4)) << 8 | 0xFF;
        term bgcolor = term_get_tuple_element(req, 5);
        if (bgcolor == context_make_atom(ctx, "\xB"
                                              "transparent")) {
            item->brcolor = 0;
        } else {
            item->brcolor = ((uint32_t) term_to_int(bgcolor)) << 8 | 0xFF;
        }

        term font = term_get_tuple_element(req, 3);
        if (font != context_make_atom(ctx, "\xB"
                                           "default16px")) {
            fprintf(stderr, "unsupported font: ");
            term_display(stderr, font, ctx);
            fprintf(stderr, "\n");
            //return;
        }

        term text_term = term_get_tuple_element(req, 6);
        int ok;
        item->data.text_data.text = interop_term_to_string(text_term, &ok);
        if (!ok) {
            fprintf(stderr, "invalid text.\n");
            return;
        }

        item->height = 16;
        item->width = strlen(item->data.text_data.text) * 8;

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
