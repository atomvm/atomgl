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

#include <limits.h>
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
};

typedef struct BaseDisplayItem BaseDisplayItem;

typedef enum
{
    DisplayItemsOk = 0,
    DisplayItemsNotAProperList,
    DisplayItemsOutOfMemory
} display_items_result_t;

static inline int display_items_scaled_cropped_mirror_width(const BaseDisplayItem *item)
{
    int64_t max = (int64_t) (item->data.image_data_with_size.width - item->source_x) * item->x_scale;
    return (item->width < max) ? item->width : (int) max;
}

static inline int display_items_scaled_cropped_mirror_height(const BaseDisplayItem *item)
{
    int64_t max = (int64_t) (item->data.image_data_with_size.height - item->source_y) * item->y_scale;
    return (item->height < max) ? item->height : (int) max;
}

static inline int display_items_scaled_cropped_col(const BaseDisplayItem *item, int col_px)
{
    int max_col = item->data.image_data_with_size.width - item->source_x - 1;
    if (item->flip_x) {
        col_px = display_items_scaled_cropped_mirror_width(item) - 1 - col_px;
    }
    int col = (col_px < 0) ? 0 : col_px / item->x_scale;
    return (col > max_col) ? max_col : col;
}

static inline int display_items_scaled_cropped_row(const BaseDisplayItem *item, int row_px)
{
    int max_row = item->data.image_data_with_size.height - item->source_y - 1;
    if (item->flip_y) {
        row_px = display_items_scaled_cropped_mirror_height(item) - 1 - row_px;
    }
    int row = (row_px < 0) ? 0 : row_px / item->y_scale;
    return (row > max_row) ? max_row : row;
}

static inline const uint32_t *display_items_scaled_cropped_pixel(const BaseDisplayItem *item,
    int col_px, int row_px)
{
    int img_width = item->data.image_data_with_size.width;
    int row = display_items_scaled_cropped_row(item, row_px);
    int col = display_items_scaled_cropped_col(item, col_px);
    return ((const uint32_t *) item->data.image_data_with_size.pix)
        + (item->source_y + row) * img_width + item->source_x + col;
}

struct ScaledCroppedRow
{
    const uint32_t *origin;
    int col_offset;
    int col_divisor;
    int clamp_px;
};

static inline void display_items_scaled_cropped_row_init(struct ScaledCroppedRow *src,
    const BaseDisplayItem *item, int row_px)
{
    int img_width = item->data.image_data_with_size.width;
    int x_scale = item->x_scale;
    int row = display_items_scaled_cropped_row(item, row_px);
    const uint32_t *row_start = ((const uint32_t *) item->data.image_data_with_size.pix)
        + (item->source_y + row) * img_width + item->source_x;
    if (item->flip_x) {
        int mirror = display_items_scaled_cropped_mirror_width(item);
        int last = (mirror > 0) ? mirror - 1 : 0;
        src->origin = row_start + last / x_scale;
        src->col_offset = x_scale - 1 - last % x_scale;
        src->col_divisor = -x_scale;
        src->clamp_px = (mirror > 0) ? mirror : 1;
    } else {
        int64_t clamp_px = (int64_t) (img_width - item->source_x) * x_scale;
        src->origin = row_start;
        src->col_offset = 0;
        src->col_divisor = x_scale;
        src->clamp_px = (clamp_px > INT_MAX) ? INT_MAX : (int) clamp_px;
    }
}

static inline int display_items_scaled_cropped_span(struct ScaledCroppedRow *src, int col_px,
    int end)
{
    if (col_px < src->clamp_px) {
        return (end < src->clamp_px) ? end : src->clamp_px;
    }
    src->origin += (src->clamp_px - 1 + src->col_offset) / src->col_divisor;
    src->col_offset = 0;
    src->col_divisor = INT_MAX;
    src->clamp_px = INT_MAX;
    return end;
}

static inline const uint32_t *display_items_scaled_cropped_row_pixel(
    const struct ScaledCroppedRow *src, int col_px)
{
    return src->origin + (col_px + src->col_offset) / src->col_divisor;
}

void display_items_init_item(BaseDisplayItem *item, term req, Context *ctx);

display_items_result_t display_items_new_list(term display_list, BaseDisplayItem **items, size_t *items_len, Context *ctx);
void display_items_delete(BaseDisplayItem items[], size_t items_len);

#endif
