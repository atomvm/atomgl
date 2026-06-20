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
    bool rgb565_pixels;
};

typedef struct BaseDisplayItem BaseDisplayItem;

void display_items_init_item(BaseDisplayItem *item, term req, Context *ctx);
void display_items_delete(BaseDisplayItem items[], size_t items_len);

#endif

