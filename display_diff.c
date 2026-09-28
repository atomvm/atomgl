/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 Davide Bettio <davide@uninstall.it>
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

#include "display_diff.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "shape.h"

bool display_diff_items_equal(const BaseDisplayItem *a, const BaseDisplayItem *b)
{
    if (a->primitive != b->primitive || a->x != b->x || a->y != b->y || a->width != b->width
        || a->height != b->height || a->brcolor != b->brcolor) {
        return false;
    }

    switch (a->primitive) {
        case PrimitiveInvalid:
        case PrimitiveRect:
            return true;

        case PrimitiveText: {
            if (a->data.text_data.fgcolor != b->data.text_data.fgcolor) {
                return false;
            }
            const char *text_a = a->data.text_data.text;
            const char *text_b = b->data.text_data.text;
            if (text_a == text_b) {
                return true;
            }
            if (text_a == NULL || text_b == NULL) {
                return false;
            }
            return strcmp(text_a, text_b) == 0;
        }

        case PrimitiveImage:
            // TODO: ufont text is rasterized into a new buffer on every update, so it never
            // compares equal; keep the source string and font handle in the item and compare
            // those instead.
            return a->data.image_data.pix == b->data.image_data.pix;

        case PrimitiveScaledCroppedImage: {
            const struct ImageDataWithSize *img_a = &a->data.image_data_with_size;
            const struct ImageDataWithSize *img_b = &b->data.image_data_with_size;
            return a->source_x == b->source_x && a->source_y == b->source_y
                && a->x_scale == b->x_scale && a->y_scale == b->y_scale
                && a->flip_x == b->flip_x && a->flip_y == b->flip_y
                && img_a->width == img_b->width && img_a->height == img_b->height
                && img_a->pix == img_b->pix;
        }

        case PrimitiveShape: {
            const struct ShapeData *shape_a = a->data.shape_data.shape;
            const struct ShapeData *shape_b = b->data.shape_data.shape;
            if (shape_a == NULL || shape_b == NULL) {
                return shape_a == shape_b;
            }
            return shape_equal(shape_a, shape_b);
        }
    }

    return false;
}

static size_t find_equal(const BaseDisplayItem items[], size_t from, size_t items_len,
    const BaseDisplayItem *item)
{
    for (size_t i = from; i < items_len; i++) {
        if (display_diff_items_equal(&items[i], item)) {
            return i;
        }
    }

    return items_len;
}

static void add_item(struct DisplayDamage *damage, const BaseDisplayItem *item)
{
    display_damage_add_rect(damage, item->x, item->y, item->width, item->height);
}

void display_diff_compute(const BaseDisplayItem old_items[], size_t old_items_len,
    const BaseDisplayItem new_items[], size_t new_items_len, struct DisplayDamage *damage)
{
    size_t i = 0;
    size_t j = 0;

    while (i < new_items_len && j < old_items_len) {
        if (display_diff_items_equal(&new_items[i], &old_items[j])) {
            i++;
            j++;
            continue;
        }

        size_t k_old = find_equal(old_items, j + 1, old_items_len, &new_items[i]);
        size_t k_new = find_equal(new_items, i + 1, new_items_len, &old_items[j]);
        bool in_old = k_old < old_items_len;
        bool in_new = k_new < new_items_len;

        if (!in_old && !in_new) {
            add_item(damage, &old_items[j]);
            add_item(damage, &new_items[i]);
            i++;
            j++;
        } else if (in_old && (!in_new || k_old - j <= k_new - i)) {
            for (; j < k_old; j++) {
                add_item(damage, &old_items[j]);
            }
        } else {
            for (; i < k_new; i++) {
                add_item(damage, &new_items[i]);
            }
        }
    }

    for (; i < new_items_len; i++) {
        add_item(damage, &new_items[i]);
    }
    for (; j < old_items_len; j++) {
        add_item(damage, &old_items[j]);
    }
}
