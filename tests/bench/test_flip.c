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
#include <string.h>

#include "display_items.h"

static int failures = 0;

static void check_in_bounds(const char *name, BaseDisplayItem *item, const uint32_t *pix,
    int img_width, int img_height)
{
    const uint32_t *lo = pix;
    const uint32_t *hi = pix + (size_t) img_width * (size_t) img_height;

    for (int row_px = 0; row_px < item->height; row_px++) {
        for (int col_px = 0; col_px < item->width; col_px++) {
            const uint32_t *p = display_items_scaled_cropped_pixel(item, col_px, row_px);
            if (p < lo || p >= hi) {
                fprintf(stderr, "%s: (%d,%d) -> pointer %ld outside [0, %d)\n", name, col_px,
                    row_px, (long) (p - pix), img_width * img_height);
                failures++;
            }
        }
    }
}

static BaseDisplayItem make_item(int width, int height, int source_x, int source_y, int x_scale,
    int y_scale, bool flip_x, bool flip_y, int img_width, int img_height, const uint32_t *pix)
{
    BaseDisplayItem item;
    memset(&item, 0, sizeof(item));
    item.primitive = PrimitiveScaledCroppedImage;
    item.width = width;
    item.height = height;
    item.source_x = source_x;
    item.source_y = source_y;
    item.x_scale = x_scale;
    item.y_scale = y_scale;
    item.flip_x = flip_x;
    item.flip_y = flip_y;
    item.data.image_data_with_size.width = img_width;
    item.data.image_data_with_size.height = img_height;
    item.data.image_data_with_size.pix = (const char *) pix;
    return item;
}

static void check_row_walk(const char *name, BaseDisplayItem *item)
{
    for (int row_px = 0; row_px < item->height; row_px++) {
        for (int start = 0; start < item->width; start++) {
            struct ScaledCroppedRow src;
            display_items_scaled_cropped_row_init(&src, item, row_px);
            int col_px = start;
            while (col_px < item->width) {
                int span_end = display_items_scaled_cropped_span(&src, col_px, item->width);
                if (span_end <= col_px || span_end > item->width) {
                    fprintf(stderr, "%s: bad span end %d at %d\n", name, span_end, col_px);
                    failures++;
                    return;
                }
                for (; col_px < span_end; col_px++) {
                    const uint32_t *p = display_items_scaled_cropped_row_pixel(&src, col_px);
                    const uint32_t *expected = display_items_scaled_cropped_pixel(item, col_px, row_px);
                    if (p != expected) {
                        fprintf(stderr, "%s: (%d,%d) walked from %d is %ld pixels off\n", name,
                            col_px, row_px, start, (long) (p - expected));
                        failures++;
                        return;
                    }
                }
            }
        }
    }
}

static void check_mirror(const char *name, const BaseDisplayItem *item)
{
    BaseDisplayItem plain = *item;
    plain.flip_x = false;
    plain.flip_y = false;
    for (int row_px = 0; row_px < item->height; row_px++) {
        for (int col_px = 0; col_px < item->width; col_px++) {
            int mirror_col = item->flip_x ? item->width - 1 - col_px : col_px;
            int mirror_row = item->flip_y ? item->height - 1 - row_px : row_px;
            const uint32_t *p = display_items_scaled_cropped_pixel(item, col_px, row_px);
            const uint32_t *expected = display_items_scaled_cropped_pixel(&plain, mirror_col, mirror_row);
            if (p != expected) {
                fprintf(stderr, "%s: (%d,%d) is not the mirror of (%d,%d)\n", name, col_px, row_px,
                    mirror_col, mirror_row);
                failures++;
                return;
            }
        }
    }
}

static uint32_t rng_state = 12345;

static int rng_range(int lo, int hi)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return lo + (int) ((rng_state >> 8) % (uint32_t) (hi - lo + 1));
}

static void test_row_walk_random(void)
{
    static uint32_t pix[24 * 24];
    for (int i = 0; i < 3000; i++) {
        int img_width = rng_range(1, 24);
        int img_height = rng_range(1, 24);
        int source_x = rng_range(0, img_width - 1);
        int source_y = rng_range(0, img_height - 1);
        int x_scale = rng_range(1, 4);
        int y_scale = rng_range(1, 4);
        int width;
        int height;
        switch (rng_range(0, 2)) {
            case 0:
                width = (img_width - source_x) * x_scale;
                height = (img_height - source_y) * y_scale;
                break;
            case 1:
                width = (img_width - source_x) * x_scale + rng_range(0, x_scale - 1);
                height = (img_height - source_y) * y_scale + rng_range(0, y_scale - 1);
                break;
            default:
                width = rng_range(1, 110);
                height = rng_range(1, 30);
                break;
        }
        BaseDisplayItem item = make_item(width, height, source_x, source_y, x_scale, y_scale,
            rng_range(0, 1), rng_range(0, 1), img_width, img_height, pix);
        int before = failures;
        check_in_bounds("random item", &item, pix, img_width, img_height);
        check_row_walk("random item", &item);
        if (width <= (img_width - source_x) * x_scale && height <= (img_height - source_y) * y_scale) {
            check_mirror("random item", &item);
        }
        if (failures != before) {
            fprintf(stderr, "  item %dx%d src (%d,%d) scale %dx%d flip %d/%d image %dx%d\n", width,
                height, source_x, source_y, x_scale, y_scale, item.flip_x, item.flip_y, img_width,
                img_height);
            return;
        }
    }
}

int main(void)
{
    {
        static const uint32_t pix[4] = { 0 };
        BaseDisplayItem item = make_item(7, 1, 1, 0, 2, 1, false, false, 4, 1, pix);
        check_in_bounds("horizontal unflipped over-read", &item, pix, 4, 1);
        check_row_walk("horizontal unflipped over-read", &item);
    }

    {
        static const uint32_t pix[4] = { 0 };
        BaseDisplayItem item = make_item(7, 1, 0, 0, 2, 1, true, false, 4, 1, pix);
        check_in_bounds("horizontal flip_x under-read", &item, pix, 4, 1);
        check_row_walk("horizontal flip_x under-read", &item);
    }

    {
        static const uint32_t pix[4] = { 0 };
        BaseDisplayItem item = make_item(1, 7, 0, 1, 1, 2, false, false, 1, 4, pix);
        check_in_bounds("vertical unflipped over-read", &item, pix, 1, 4);
        check_row_walk("vertical unflipped over-read", &item);
    }

    {
        static const uint32_t pix[4] = { 0 };
        BaseDisplayItem item = make_item(1, 7, 0, 0, 1, 1, false, true, 1, 4, pix);
        check_in_bounds("vertical flip_y under-read", &item, pix, 1, 4);
        check_row_walk("vertical flip_y under-read", &item);
    }

    {
        static const uint32_t pix[16 * 8] = { 0 };
        BaseDisplayItem item = make_item(20, 10, 3, 1, 2, 2, true, true, 16, 8, pix);
        check_in_bounds("both axes flipped, cropped and scaled", &item, pix, 16, 8);
        check_row_walk("both axes flipped, cropped and scaled", &item);
    }

    {
        static const uint32_t pix[3] = { 0 };
        BaseDisplayItem item = make_item(5, 1, 0, 0, 2, 1, true, false, 3, 1, pix);
        static const int expected[5] = { 2, 1, 1, 0, 0 };
        for (int c = 0; c < 5; c++) {
            if (display_items_scaled_cropped_pixel(&item, c, 0) != pix + expected[c]) {
                fprintf(stderr, "odd width flip: column %d shows %ld, expected %d\n", c,
                    (long) (display_items_scaled_cropped_pixel(&item, c, 0) - pix), expected[c]);
                failures++;
            }
        }
        check_row_walk("odd width flip", &item);
    }

    test_row_walk_random();

    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    printf("all flip clamp tests passed\n");
    return 0;
}
