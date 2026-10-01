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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "damage.h"

static int failures;
static int checks;

#define CHECK(cond, ...)                                         \
    do {                                                         \
        checks++;                                                \
        if (!(cond)) {                                           \
            failures++;                                          \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                        \
            fprintf(stderr, "\n");                               \
        }                                                        \
    } while (0)

static uint32_t pixels_a[4 * 2];
static uint32_t pixels_b[4 * 2];

static BaseDisplayItem rect(int x, int y, int width, int height)
{
    BaseDisplayItem item;
    memset(&item, 0, sizeof(item));
    item.primitive = PrimitiveRect;
    item.x = x;
    item.y = y;
    item.width = width;
    item.height = height;
    item.brcolor = 0xFF0000FF;

    return item;
}

static BaseDisplayItem scaled_cropped_image(const uint32_t *pixels)
{
    BaseDisplayItem item;
    memset(&item, 0, sizeof(item));
    item.primitive = PrimitiveScaledCroppedImage;
    item.x = 10;
    item.y = 20;
    item.width = 8;
    item.height = 4;
    item.x_scale = 2;
    item.y_scale = 2;
    item.data.image_data_with_size.width = 4;
    item.data.image_data_with_size.height = 2;
    item.data.image_data_with_size.pix = (const char *) pixels;

    return item;
}

static BaseDisplayItem *items_new(const BaseDisplayItem items[], int items_len)
{
    BaseDisplayItem *copy = malloc(sizeof(BaseDisplayItem) * items_len);
    if (copy == NULL) {
        abort();
    }
    memcpy(copy, items, sizeof(BaseDisplayItem) * items_len);

    return copy;
}

static struct Rectangle diff(const BaseDisplayItem orig[], int orig_len,
    const BaseDisplayItem new[], int new_len)
{
    BaseDisplayItem *orig_copy = items_new(orig, orig_len);
    BaseDisplayItem *new_copy = items_new(new, new_len);
    struct Rectangle damaged;
    memset(&damaged, 0, sizeof(damaged));
    damage_diff(orig_copy, orig_len, new_copy, new_len, &damaged);
    free(orig_copy);
    free(new_copy);

    return damaged;
}

static void test_same_list(void)
{
    BaseDisplayItem items[] = { rect(0, 0, 4, 4), scaled_cropped_image(pixels_a) };
    struct Rectangle damaged = diff(items, 2, items, 2);
    CHECK(!damaged.valid, "same list: damage (%d, %d, %d, %d)", damaged.x, damaged.y,
        damaged.width, damaged.height);
}

static void test_scaled_cropped_image_changed(void)
{
    BaseDisplayItem orig[] = { scaled_cropped_image(pixels_a) };
    BaseDisplayItem new[] = { scaled_cropped_image(pixels_b) };
    struct Rectangle damaged = diff(orig, 1, new, 1);
    CHECK(damaged.valid && damaged.x == 10 && damaged.y == 20 && damaged.width == 8
            && damaged.height == 4,
        "other image of the same size: valid %d damage (%d, %d, %d, %d)", damaged.valid,
        damaged.x, damaged.y, damaged.width, damaged.height);
}

static void test_longer_list(void)
{
    BaseDisplayItem orig[] = { rect(0, 0, 4, 4) };
    BaseDisplayItem new[] = { rect(0, 0, 4, 4), rect(5, 6, 7, 8) };
    struct Rectangle damaged = diff(orig, 1, new, 2);
    CHECK(damaged.valid && damaged.x == 5 && damaged.y == 6 && damaged.width == 7
            && damaged.height == 8,
        "item added at the end: valid %d damage (%d, %d, %d, %d)", damaged.valid, damaged.x,
        damaged.y, damaged.width, damaged.height);
}

int main(void)
{
    test_same_list();
    test_scaled_cropped_image_changed();
    test_longer_list();

    if (failures) {
        fprintf(stderr, "%d of %d checks failed\n", failures, checks);
        return 1;
    }
    printf("all %d damage checks passed\n", checks);

    return 0;
}
