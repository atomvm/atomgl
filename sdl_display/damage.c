/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 Tom Hoenderdos <tomhoenderdos@gmail.com>
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

#include "damage.h"

#include <string.h>

static inline int int_min(int a, int b)
{
    return (a > b) ? b : a;
}

static inline int int_max(int a, int b)
{
    return (a > b) ? a : b;
}

static bool cmp_display_item(BaseDisplayItem *a, BaseDisplayItem *b)
{
    if (a->primitive != b->primitive || a->x != b->x || a->y != b->y ||
            a->width != b->width || a->height != b->height || a->brcolor != b->brcolor) {
        return false;
    }

    switch (a->primitive) {
        case PrimitiveImage:
            return a->data.image_data.pix == b->data.image_data.pix;

        case PrimitiveRect:
            return true;

        case PrimitiveText:
            return (a->data.text_data.fgcolor == b->data.text_data.fgcolor) &&
                !strcmp(a->data.text_data.text, b->data.text_data.text);

        case PrimitiveScaledCroppedImage:
            return (a->data.image_data_with_size.pix == b->data.image_data_with_size.pix)
                && (a->data.image_data_with_size.width == b->data.image_data_with_size.width)
                && (a->data.image_data_with_size.height == b->data.image_data_with_size.height)
                && (a->x_scale == b->x_scale) && (a->y_scale == b->y_scale)
                && (a->source_x == b->source_x) && (a->source_y == b->source_y);

        default: {
            return true;
        }
    }
}

static void update_damaged_area(struct Rectangle *area, const struct Rectangle *damage)
{
    if (area->valid) {
        int right = int_max(area->x + area->width, damage->x + damage->width);
        int bottom = int_max(area->y + area->height, damage->y + damage->height);
        area->x = int_min(area->x, damage->x);
        area->y = int_min(area->y, damage->y);
        area->width = right - area->x;
        area->height = bottom - area->y;
    } else {
        area->x = damage->x;
        area->y = damage->y;
        area->width = damage->width;
        area->height = damage->height;
        area->valid = true;
    }
}

void damage_clip(struct Rectangle *rectangle, const struct Rectangle *clip_region)
{
    int right = int_min(rectangle->x + rectangle->width, clip_region->x + clip_region->width);
    int bottom = int_min(rectangle->y + rectangle->height, clip_region->y + clip_region->height);
    rectangle->x = int_max(rectangle->x, clip_region->x);
    rectangle->y = int_max(rectangle->y, clip_region->y);
    rectangle->width = right - rectangle->x;
    rectangle->height = bottom - rectangle->y;
}

void damage_diff(BaseDisplayItem *orig, int orig_len, BaseDisplayItem *new, int new_len, struct Rectangle *damaged)
{
    if (orig_len == 0) {
        for (int i = 0; i < new_len; i++) {
            struct Rectangle irect = {
                .x = new[i].x,
                .y = new[i].y,
                .width = new[i].width,
                .height = new[i].height,
                .valid = true
            };
            update_damaged_area(damaged, &irect);
        }
        return;
    }

    int j = 0;

    for (int i = 0; i < new_len; i++) {
        if (j < orig_len && cmp_display_item(&new[i], &orig[j])) {
            j++;
        } else {
            bool found = false;
            for (int k = j + 1; k < orig_len; k++) {
                if (cmp_display_item(&new[i], &orig[k])) {
                    for (int l = j; l < k; l++) {
                        struct Rectangle irect = {
                            .x = orig[l].x,
                            .y = orig[l].y,
                            .width = orig[l].width,
                            .height = orig[l].height,
                            .valid = true
                        };
                        update_damaged_area(damaged, &irect);
                    }

                    j = k + 1;
                    found = true;
                    break;
                }
            }
            if (!found) {
                struct Rectangle irect = {
                    .x = new[i].x,
                    .y = new[i].y,
                    .width = new[i].width,
                    .height = new[i].height,
                    .valid = true
                };
                update_damaged_area(damaged, &irect);
            }
        }
    }

    for (int l = j; l < orig_len; l++) {
        struct Rectangle irect = {
            .x = orig[l].x,
            .y = orig[l].y,
            .width = orig[l].width,
            .height = orig[l].height,
            .valid = true
        };
        update_damaged_area(damaged, &irect);
    }
}
