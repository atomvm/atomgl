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

void display_items_init_item(BaseDisplayItem *item, term req, Context *ctx);

display_items_result_t display_items_new_list(term display_list, BaseDisplayItem **items, size_t *items_len, Context *ctx);
void display_items_delete(BaseDisplayItem items[], size_t items_len);

#endif
