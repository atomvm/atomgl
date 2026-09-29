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

#ifndef _DISPLAY_ITEMS_H_
#define _DISPLAY_ITEMS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <context.h>

// TODO: deprecated helper, remove this
static inline term context_make_atom(Context *ctx, AtomString string)
{
    return globalcontext_make_atom(ctx->global, string);
}

#define DISPLAY_ITEMS_COORD_LIMIT 32767

typedef enum
{
    PrimitiveInvalid = 0,
    PrimitiveImage,
    PrimitiveScaledCroppedImage,
    PrimitiveRect,
    PrimitiveText
} primitive_t;

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
    primitive_t primitive;
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
    bool flip_x;
    bool flip_y;

    bool owns_data;

    // Rendering scratch, never compared: the next item that covers the row being drawn, see
    // display_items_row()
    struct BaseDisplayItem *next;
};

typedef struct BaseDisplayItem BaseDisplayItem;

typedef enum
{
    DisplayItemsOk = 0,
    DisplayItemsNotAProperList,
    DisplayItemsOutOfMemory
} display_items_result_t;

// Links the items that cover row ypos through their next field, in display list order, and
// returns the first of them, or NULL if none does.
static inline BaseDisplayItem *display_items_row(BaseDisplayItem items[], size_t items_len, int ypos)
{
    BaseDisplayItem *head = NULL;
    BaseDisplayItem **link = &head;
    for (size_t i = 0; i < items_len; i++) {
        BaseDisplayItem *item = &items[i];
        if (ypos >= item->y && ypos < item->y + item->height) {
            *link = item;
            link = &item->next;
        }
    }
    *link = NULL;
    return head;
}

static inline uint8_t rgba8888_get_alpha(uint32_t color)
{
    return color & 0xFF;
}

// A scaled_cropped_image shows display pixel px of an axis (counted from the item's x or y)
// from source pixel px / scale of the cropped image (counted from source_x or source_y). With
// flip_x or flip_y the axis is mirrored within the part of the item the image covers, which is
// all of it unless the item is wider or taller than the scaled image. Past the end of the image
// the edge source pixel repeats.
struct ScaledCroppedAxis
{
    int last_px; // with flip, the last display pixel the image covers, mirrored to 0
    int scale;
    int last_src; // the last source pixel of the cropped image
    bool flip;
};

static inline void display_items_scaled_cropped_axis_init(struct ScaledCroppedAxis *axis, int len,
    int src_len, int scale, bool flip)
{
    int64_t covered = (int64_t) src_len * scale;
    axis->last_px = ((len < covered) ? len : (int) covered) - 1;
    axis->scale = scale;
    axis->last_src = src_len - 1;
    axis->flip = flip;
}

static inline int display_items_scaled_cropped_axis_src(const struct ScaledCroppedAxis *axis,
    int px)
{
    if (axis->flip) {
        px = axis->last_px - px;
        // Past the mirrored image: the edge pixel, source pixel 0
        if (px < 0) {
            px = 0;
        }
    }
    int src = px / axis->scale;
    return (src < axis->last_src) ? src : axis->last_src;
}

// One display row of a scaled_cropped_image: its source row and the mapping of its columns.
struct ScaledCroppedRow
{
    const uint32_t *pixels; // the source row, from source_x
    struct ScaledCroppedAxis cols;
};

static inline void display_items_scaled_cropped_row_init(struct ScaledCroppedRow *row,
    const BaseDisplayItem *item, int row_px)
{
    const struct ImageDataWithSize *img = &item->data.image_data_with_size;
    struct ScaledCroppedAxis rows;
    display_items_scaled_cropped_axis_init(&rows, item->height, img->height - item->source_y,
        item->y_scale, item->flip_y);
    int src_row = display_items_scaled_cropped_axis_src(&rows, row_px);
    row->pixels = ((const uint32_t *) img->pix) + (item->source_y + src_row) * img->width
        + item->source_x;
    display_items_scaled_cropped_axis_init(&row->cols, item->width, img->width - item->source_x,
        item->x_scale, item->flip_x);
}

static inline const uint32_t *display_items_scaled_cropped_row_pixel(
    const struct ScaledCroppedRow *row, int col_px)
{
    return row->pixels + display_items_scaled_cropped_axis_src(&row->cols, col_px);
}

void display_items_init_item(BaseDisplayItem *item, term req, Context *ctx);

display_items_result_t display_items_new_list(term display_list, BaseDisplayItem **items, size_t *items_len, Context *ctx);
void display_items_delete(BaseDisplayItem items[], size_t items_len);

#endif
