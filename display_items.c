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

#include <defaultatoms.h>
#include <interop.h>
#include <utils.h>

#include "font_data.h"

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

    uint32_t *pixel = (uint32_t *) (((uint8_t *) surface->buffer)
            + (surface->width * ypos + xpos) * sizeof(uint32_t));

    // The `color` parameter is the LUT-mapped glyph value from
    // draw_char: 0 = full foreground (fg_color=0 in default props),
    // 240 = full background (bg_color=15), in steps of 16. Render
    // the foreground RGB on transparent with anti-aliased alpha
    // derived from the inverted grayscale.
    uint8_t alpha = (15 - (color >> 4)) * 17;
    *pixel = ((uint32_t) alpha << 24) | (surface->fg_color & 0x00FFFFFFu);
}
#endif /* ENABLE_UFONT */

#define MAX_LOGGED_INVALID_ITEMS 3

static bool get_int_element(term req, int index, avm_int64_t *out)
{
    term t = term_get_tuple_element(req, index);
    if (UNLIKELY(!term_is_int64(t))) {
        return false;
    }
    *out = term_to_int64(t);

    return true;
}

static bool get_bounded_element(term req, int index, int min, int max, int *out)
{
    avm_int64_t v;
    if (UNLIKELY(!get_int_element(req, index, &v) || v < min || v > max)) {
        return false;
    }
    *out = (int) v;

    return true;
}

static bool get_coord_element(term req, int index, int *out)
{
    return get_bounded_element(req, index, -DISPLAY_ITEMS_COORD_LIMIT, DISPLAY_ITEMS_COORD_LIMIT, out);
}

static bool get_shape_value_element(term req, int index, int *out)
{
    return get_bounded_element(req, index, -SHAPE_VALUE_LIMIT, SHAPE_VALUE_LIMIT, out);
}

static bool get_color_element(term req, int index, uint32_t *out)
{
    avm_int64_t color;
    if (UNLIKELY(!get_int_element(req, index, &color))) {
        return false;
    }
    *out = ((uint32_t) color) << 8 | 0xFF;

    return true;
}

static bool get_bgcolor_element(term req, int index, uint32_t *out, Context *ctx)
{
    if (term_get_tuple_element(req, index) == globalcontext_make_atom(ctx->global, ATOM_STR("\xB", "transparent"))) {
        *out = 0;
        return true;
    }

    return get_color_element(req, index, out);
}

static bool get_rgba8888_image(term img, struct ImageDataWithSize *out, Context *ctx)
{
    if (UNLIKELY(!term_is_tuple(img) || term_get_tuple_arity(img) != 4
            || term_get_tuple_element(img, 0) != globalcontext_make_atom(ctx->global, ATOM_STR("\x8", "rgba8888")))) {
        return false;
    }
    int width;
    int height;
    if (UNLIKELY(!get_bounded_element(img, 1, 1, DISPLAY_ITEMS_COORD_LIMIT, &width)
            || !get_bounded_element(img, 2, 1, DISPLAY_ITEMS_COORD_LIMIT, &height))) {
        return false;
    }
    term pixels = term_get_tuple_element(img, 3);
    if (UNLIKELY(!term_is_binary(pixels) || term_binary_size(pixels) < (uint64_t) width * height * 4)) {
        return false;
    }
    out->width = width;
    out->height = height;
    out->pix = term_binary_data(pixels);

    return true;
}

static const char *init_image_item(BaseDisplayItem *item, term req, Context *ctx)
{
    struct ImageDataWithSize img;
    if (UNLIKELY(term_get_tuple_arity(req) != 5)) {
        return "wrong arity";
    }
    if (UNLIKELY(!get_coord_element(req, 1, &item->x) || !get_coord_element(req, 2, &item->y)
            || !get_bgcolor_element(req, 3, &item->brcolor, ctx))) {
        return "bad position or color";
    }
    if (UNLIKELY(!get_rgba8888_image(term_get_tuple_element(req, 4), &img, ctx))) {
        return "bad image";
    }
    item->primitive = PrimitiveImage;
    item->width = img.width;
    item->height = img.height;
    item->data.image_data.pix = img.pix;

    return NULL;
}

static const char *init_scaled_cropped_image_item(BaseDisplayItem *item, term req, Context *ctx)
{
    struct ImageDataWithSize img;
    if (UNLIKELY(term_get_tuple_arity(req) != 12)) {
        return "wrong arity";
    }
    if (UNLIKELY(!get_coord_element(req, 1, &item->x) || !get_coord_element(req, 2, &item->y)
            || !get_bounded_element(req, 3, 0, DISPLAY_ITEMS_COORD_LIMIT, &item->width)
            || !get_bounded_element(req, 4, 0, DISPLAY_ITEMS_COORD_LIMIT, &item->height)
            || !get_bgcolor_element(req, 5, &item->brcolor, ctx)
            || !get_bounded_element(req, 6, 0, DISPLAY_ITEMS_COORD_LIMIT, &item->source_x)
            || !get_bounded_element(req, 7, 0, DISPLAY_ITEMS_COORD_LIMIT, &item->source_y)
            || !get_bounded_element(req, 8, 1, DISPLAY_ITEMS_COORD_LIMIT, &item->x_scale)
            || !get_bounded_element(req, 9, 1, DISPLAY_ITEMS_COORD_LIMIT, &item->y_scale))) {
        return "bad position, size, color, source or scale";
    }
    if (UNLIKELY(!get_rgba8888_image(term_get_tuple_element(req, 11), &img, ctx))) {
        return "bad image";
    }
    if (UNLIKELY(item->source_x >= img.width || item->source_y >= img.height)) {
        return "source outside the image";
    }

    int max_width = (img.width - item->source_x) * item->x_scale;
    int max_height = (img.height - item->source_y) * item->y_scale;
    if (item->width > max_width) {
        item->width = max_width;
    }
    if (item->height > max_height) {
        item->height = max_height;
    }

    term flip_x = globalcontext_make_atom(ctx->global, ATOM_STR("\x6", "flip_x"));
    term flip_y = globalcontext_make_atom(ctx->global, ATOM_STR("\x6", "flip_y"));
    term opts = term_get_tuple_element(req, 10);
    while (term_is_nonempty_list(opts)) {
        term opt = term_get_list_head(opts);
        if (term_is_tuple(opt) && term_get_tuple_arity(opt) == 2
            && term_get_tuple_element(opt, 1) == TRUE_ATOM) {
            opt = term_get_tuple_element(opt, 0);
        }
        if (opt == flip_x) {
            item->flip_x = true;
        } else if (opt == flip_y) {
            item->flip_y = true;
        }
        opts = term_get_list_tail(opts);
    }

    item->primitive = PrimitiveScaledCroppedImage;
    item->data.image_data_with_size = img;

    return NULL;
}

static const char *init_shape_item(BaseDisplayItem *item, struct ShapeData *shape, uint32_t color)
{
    if (IS_NULL_PTR(shape)) {
        return "out of memory";
    }
    item->primitive = PrimitiveShape;
    item->brcolor = color;
    item->data.shape_data.shape = shape;
    shape_bounds(shape, &item->x, &item->y, &item->width, &item->height);

    return NULL;
}

static const char *init_rounded_rect_item(BaseDisplayItem *item, term req)
{
    int x, y, width, height, radius;
    uint32_t color;
    if (UNLIKELY(term_get_tuple_arity(req) != 7)) {
        return "wrong arity";
    }
    if (UNLIKELY(!get_shape_value_element(req, 1, &x) || !get_shape_value_element(req, 2, &y)
            || !get_bounded_element(req, 3, 1, SHAPE_VALUE_LIMIT, &width)
            || !get_bounded_element(req, 4, 1, SHAPE_VALUE_LIMIT, &height)
            || !get_bounded_element(req, 5, 0, SHAPE_VALUE_LIMIT, &radius)
            || !get_color_element(req, 6, &color))) {
        return "bad position, size, radius or color";
    }

    return init_shape_item(item, shape_new_rounded_rect(x, y, width, height, radius), color);
}

static const char *init_circle_item(BaseDisplayItem *item, term req)
{
    int cx, cy, radius;
    uint32_t color;
    if (UNLIKELY(term_get_tuple_arity(req) != 5)) {
        return "wrong arity";
    }
    if (UNLIKELY(!get_shape_value_element(req, 1, &cx) || !get_shape_value_element(req, 2, &cy)
            || !get_bounded_element(req, 3, 1, SHAPE_VALUE_LIMIT, &radius)
            || !get_color_element(req, 4, &color))) {
        return "bad center, radius or color";
    }

    return init_shape_item(item, shape_new_ellipse(cx, cy, radius, radius), color);
}

static const char *init_ellipse_item(BaseDisplayItem *item, term req)
{
    int cx, cy, rx, ry;
    uint32_t color;
    if (UNLIKELY(term_get_tuple_arity(req) != 6)) {
        return "wrong arity";
    }
    if (UNLIKELY(!get_shape_value_element(req, 1, &cx) || !get_shape_value_element(req, 2, &cy)
            || !get_bounded_element(req, 3, 1, SHAPE_VALUE_LIMIT, &rx)
            || !get_bounded_element(req, 4, 1, SHAPE_VALUE_LIMIT, &ry)
            || !get_color_element(req, 5, &color))) {
        return "bad center, radii or color";
    }

    return init_shape_item(item, shape_new_ellipse(cx, cy, rx, ry), color);
}

static const char *init_line_item(BaseDisplayItem *item, term req)
{
    int x1, y1, x2, y2, thickness;
    uint32_t color;
    if (UNLIKELY(term_get_tuple_arity(req) != 7)) {
        return "wrong arity";
    }
    if (UNLIKELY(!get_shape_value_element(req, 1, &x1) || !get_shape_value_element(req, 2, &y1)
            || !get_shape_value_element(req, 3, &x2) || !get_shape_value_element(req, 4, &y2)
            || !get_bounded_element(req, 5, 1, SHAPE_VALUE_LIMIT, &thickness)
            || !get_color_element(req, 6, &color))) {
        return "bad points, thickness or color";
    }

    return init_shape_item(item, shape_new_line(x1, y1, x2, y2, thickness), color);
}

static int clamp_coord(avm_int64_t v)
{
    if (v < -DISPLAY_ITEMS_COORD_LIMIT) {
        return -DISPLAY_ITEMS_COORD_LIMIT;
    }
    if (v > DISPLAY_ITEMS_COORD_LIMIT) {
        return DISPLAY_ITEMS_COORD_LIMIT;
    }

    return (int) v;
}

static bool get_clamped_span(term req, int start_index, int size_index, int *start, int *size)
{
    avm_int64_t s;
    avm_int64_t n;
    if (UNLIKELY(!get_int_element(req, start_index, &s) || !get_int_element(req, size_index, &n))) {
        return false;
    }
    *start = clamp_coord(s);
    if (n <= 0) {
        *size = 0;
        return true;
    }
    avm_int64_t end = (s > INT64_MAX - n) ? INT64_MAX : s + n;
    *size = clamp_coord(end) - *start;

    return true;
}

static const char *init_rect_item(BaseDisplayItem *item, term req, Context *ctx)
{
    if (UNLIKELY(term_get_tuple_arity(req) != 6)) {
        return "wrong arity";
    }
    if (UNLIKELY(!get_clamped_span(req, 1, 3, &item->x, &item->width)
            || !get_clamped_span(req, 2, 4, &item->y, &item->height)
            || !get_color_element(req, 5, &item->brcolor))) {
        return "bad position, size or color";
    }
    item->primitive = PrimitiveRect;

    return NULL;
}

static void init_default_font_text(BaseDisplayItem *item, avm_int64_t x, int y, char *text,
    uint32_t fgcolor, uint32_t brcolor)
{
    avm_int64_t len = (avm_int64_t) strlen(text);
    if (x < -DISPLAY_ITEMS_COORD_LIMIT) {
        avm_int64_t skip = (-DISPLAY_ITEMS_COORD_LIMIT - x + CHAR_WIDTH - 1) / CHAR_WIDTH;
        if (skip > len) {
            skip = len;
        }
        memmove(text, text + skip, len - skip + 1);
        len -= skip;
        x += skip * CHAR_WIDTH;
    }
    item->x = clamp_coord(x);
    avm_int64_t width = len * CHAR_WIDTH;
    if (width > DISPLAY_ITEMS_COORD_LIMIT - item->x) {
        width = DISPLAY_ITEMS_COORD_LIMIT - item->x;
    }
    item->primitive = PrimitiveText;
    item->y = y;
    item->width = (int) width;
    item->height = 16;
    item->brcolor = brcolor;
    item->data.text_data.fgcolor = fgcolor;
    item->data.text_data.text = text;
}

static const char *init_text_item(BaseDisplayItem *item, term req, Context *ctx)
{
    avm_int64_t x;
    avm_int64_t y;
    uint32_t fgcolor;
    uint32_t brcolor;
    if (UNLIKELY(term_get_tuple_arity(req) != 7)) {
        return "wrong arity";
    }
    if (UNLIKELY(!get_int_element(req, 1, &x) || !get_int_element(req, 2, &y)
            || !get_color_element(req, 4, &fgcolor) || !get_bgcolor_element(req, 5, &brcolor, ctx))) {
        return "bad position or color";
    }
    term font = term_get_tuple_element(req, 3);
    if (UNLIKELY(!term_is_atom(font))) {
        return "bad font";
    }
    term text_term = term_get_tuple_element(req, 6);
    int ok;
    char *text = interop_term_to_string(text_term, &ok);
    if (UNLIKELY(!ok || IS_NULL_PTR(text))) {
        return "bad text";
    }

    if (font == globalcontext_make_atom(ctx->global, ATOM_STR("\xB", "default16px"))) {
        init_default_font_text(item, x, clamp_coord(y), text, fgcolor, brcolor);

    } else {
#ifdef ENABLE_UFONT
        char *handle = interop_atom_to_string(ctx, font);
        EpdFont *loaded_font = NULL;
        if (!IS_NULL_PTR(handle)) {
            loaded_font = ufont_manager_find_by_handle(ufont_manager, handle);
            free(handle);
        }

        if (UNLIKELY(IS_NULL_PTR(loaded_font))) {
            free(text);
            return "unsupported font";
        }

        EpdFontProperties props = epd_font_properties_default();
        EpdRect rect = epd_get_string_rect(loaded_font, text, 0, 0, 0, &props);
        if (rect.width <= 0 || rect.height <= 0) {
            free(text);
            return NULL;
        }

        struct Surface surface;
        surface.width = rect.width;
        surface.height = rect.height;
        surface.buffer = malloc(rect.width * rect.height * BPP);
        if (IS_NULL_PTR(surface.buffer)) {
            free(text);
            return "out of memory";
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
        if (UNLIKELY(res != EPD_DRAW_SUCCESS)) {
            free(surface.buffer);
            return "text drawing failed";
        }

        item->primitive = PrimitiveImage;
        item->x = clamp_coord(x);
        item->y = clamp_coord(y);
        item->width = surface.width;
        item->height = surface.height;
        item->brcolor = brcolor;
        item->data.image_data.pix = surface.buffer;
        item->owns_data = true;
#else
        fprintf(stderr, "unsupported font: ");
        term_display(stderr, font, ctx);
        fprintf(stderr, "\n");
        init_default_font_text(item, x, clamp_coord(y), text, fgcolor, brcolor);
#endif
    }

    return NULL;
}

static const char *init_item(BaseDisplayItem *item, term req, Context *ctx)
{
    memset(item, 0, sizeof(*item));

    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        return "not a command tuple";
    }

    term cmd = term_get_tuple_element(req, 0);
    const char *reason;

    if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x5", "image"))) {
        reason = init_image_item(item, req, ctx);

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x14", "scaled_cropped_image"))) {
        reason = init_scaled_cropped_image_item(item, req, ctx);

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x4", "rect"))) {
        reason = init_rect_item(item, req, ctx);

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x4", "text"))) {
        reason = init_text_item(item, req, ctx);

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\xC", "rounded_rect"))) {
        reason = init_rounded_rect_item(item, req);

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x6", "circle"))) {
        reason = init_circle_item(item, req);

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x7", "ellipse"))) {
        reason = init_ellipse_item(item, req);

    } else if (cmd == globalcontext_make_atom(ctx->global, ATOM_STR("\x4", "line"))) {
        reason = init_line_item(item, req);

    } else {
        reason = "unknown command";
    }

    if (UNLIKELY(reason != NULL)) {
        memset(item, 0, sizeof(*item));
    }

    return reason;
}

static void log_invalid_item(term req, size_t index, const char *reason, Context *ctx)
{
    fprintf(stderr, "invalid display list item");
    if (index > 0) {
        fprintf(stderr, " %u", (unsigned) index);
    }
    if (term_is_tuple(req)) {
        int arity = term_get_tuple_arity(req);
        fprintf(stderr, " (");
        if (arity >= 1 && term_is_atom(term_get_tuple_element(req, 0))) {
            term_display(stderr, term_get_tuple_element(req, 0), ctx);
        } else {
            fprintf(stderr, "tuple");
        }
        fprintf(stderr, "/%d)", arity);
    }
    fprintf(stderr, ": %s\n", reason);
}

void display_items_init_item(BaseDisplayItem *item, term req, Context *ctx)
{
    const char *reason = init_item(item, req, ctx);
    if (UNLIKELY(reason != NULL)) {
        log_invalid_item(req, 0, reason, ctx);
    }
}

display_items_result_t display_items_new_list(term display_list, BaseDisplayItem **items, size_t *items_len, Context *ctx)
{
    int proper;
    int len = term_list_length(display_list, &proper);
    if (UNLIKELY(!proper)) {
        fprintf(stderr, "invalid display list: not a proper list\n");
        return DisplayItemsNotAProperList;
    }

    if (len == 0) {
        *items = NULL;
        *items_len = 0;
        return DisplayItemsOk;
    }

    BaseDisplayItem *new_items = malloc(sizeof(BaseDisplayItem) * len);
    if (IS_NULL_PTR(new_items)) {
        fprintf(stderr, "failed to allocate display list items\n");
        return DisplayItemsOutOfMemory;
    }

    size_t invalid = 0;
    for (int i = 0; i < len; i++) {
        term req = term_get_list_head(display_list);
        const char *reason = init_item(&new_items[i], req, ctx);
        if (UNLIKELY(reason != NULL)) {
            if (invalid < MAX_LOGGED_INVALID_ITEMS) {
                log_invalid_item(req, i + 1, reason, ctx);
            }
            invalid++;
        }
        display_list = term_get_list_tail(display_list);
    }
    if (UNLIKELY(invalid > MAX_LOGGED_INVALID_ITEMS)) {
        fprintf(stderr, "%u more invalid display list items\n", (unsigned) (invalid - MAX_LOGGED_INVALID_ITEMS));
    }

    *items = new_items;
    *items_len = len;

    return DisplayItemsOk;
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

            case PrimitiveShape:
                shape_destroy(item->data.shape_data.shape);
                break;

            default: {
                break;
            }
        }
    }

    free(items);
}
