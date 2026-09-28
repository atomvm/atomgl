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

#include "display_damage.h"

#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static int failures = 0;

#define CHECK(cond)                                                                  \
    do {                                                                             \
        if (!(cond)) {                                                               \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                                              \
        }                                                                            \
    } while (0)

static bool damage_covers_pixel(const struct DisplayDamage *damage, int x, int y)
{
    for (size_t i = 0; i < damage->rects_len; i++) {
        const struct DisplayRect *r = &damage->rects[i];
        if (x >= r->x && x < r->x + r->width && y >= r->y && y < r->y + r->height) {
            return true;
        }
    }

    return false;
}

static bool rect_equals(const struct DisplayRect *r, int x, int y, int width, int height)
{
    return r->x == x && r->y == y && r->width == width && r->height == height;
}

static void check_invariants(const struct DisplayDamage *damage)
{
    CHECK(damage->rects_len <= DISPLAY_DAMAGE_MAX_RECTS);
    for (size_t i = 0; i < damage->rects_len; i++) {
        const struct DisplayRect *r = &damage->rects[i];
        CHECK(r->width > 0 && r->height > 0);
        CHECK(r->x >= 0 && r->y >= 0);
        CHECK(r->x + r->width <= damage->screen_width);
        CHECK(r->y + r->height <= damage->screen_height);
    }
}

static void test_empty_and_degenerate(void)
{
    struct DisplayDamage damage;
    display_damage_init(&damage, 20, 10);
    CHECK(display_damage_is_empty(&damage));
    CHECK(!display_damage_is_full(&damage));

    display_damage_add_rect(&damage, 1, 1, 0, 5);
    display_damage_add_rect(&damage, 1, 1, 5, 0);
    display_damage_add_rect(&damage, 1, 1, -3, 5);
    display_damage_add_rect(&damage, 1, 1, 5, -3);
    // Fully off screen on every side.
    display_damage_add_rect(&damage, 20, 0, 5, 5);
    display_damage_add_rect(&damage, 0, 10, 5, 5);
    display_damage_add_rect(&damage, -5, 0, 5, 5);
    display_damage_add_rect(&damage, 0, -5, 5, 5);
    // Huge coordinates must not overflow.
    display_damage_add_rect(&damage, INT_MAX, INT_MAX, INT_MAX, INT_MAX);
    display_damage_add_rect(&damage, INT_MIN, INT_MIN, 10, 10);
    CHECK(display_damage_is_empty(&damage));

    int x_start;
    int x_end;
    CHECK(!display_damage_rows_span(&damage, 0, 10, &x_start, &x_end));
}

static void test_clipping(void)
{
    struct DisplayDamage damage;
    display_damage_init(&damage, 20, 10);

    display_damage_add_rect(&damage, -5, -5, 10, 10);
    CHECK(damage.rects_len == 1);
    CHECK(rect_equals(&damage.rects[0], 0, 0, 5, 5));

    display_damage_add_rect(&damage, 15, 5, 100, 100);
    CHECK(damage.rects_len == 2);
    CHECK(rect_equals(&damage.rects[1], 15, 5, 5, 5));

    display_damage_add_rect(&damage, INT_MIN, INT_MIN, INT_MAX, INT_MAX);
    CHECK(damage.rects_len == 2);
    CHECK(rect_equals(&damage.rects[1], 15, 5, 5, 5));

    display_damage_add_rect(&damage, -10, -10, 40, 40);
    CHECK(display_damage_is_full(&damage));
    check_invariants(&damage);
}

static void test_containment(void)
{
    struct DisplayDamage damage;
    display_damage_init(&damage, 100, 100);

    display_damage_add_rect(&damage, 10, 10, 20, 20);
    display_damage_add_rect(&damage, 12, 12, 5, 5);
    CHECK(damage.rects_len == 1);
    CHECK(rect_equals(&damage.rects[0], 10, 10, 20, 20));

    display_damage_add_rect(&damage, 50, 50, 5, 5);
    display_damage_add_rect(&damage, 60, 60, 5, 5);
    CHECK(damage.rects_len == 3);
    display_damage_add_rect(&damage, 40, 40, 40, 40);
    CHECK(damage.rects_len == 2);
    CHECK(damage_covers_pixel(&damage, 52, 52));
    CHECK(damage_covers_pixel(&damage, 62, 62));
    CHECK(!damage_covers_pixel(&damage, 35, 35));

    display_damage_add_all(&damage);
    CHECK(damage.rects_len == 1);
    CHECK(display_damage_is_full(&damage));
    display_damage_add_rect(&damage, 1, 2, 3, 4);
    CHECK(display_damage_is_full(&damage));

    display_damage_clear(&damage);
    CHECK(display_damage_is_empty(&damage));
    check_invariants(&damage);
}

static void test_merging(void)
{
    struct DisplayDamage damage;
    display_damage_init(&damage, 200, 200);

    // Two clusters, far apart: the merge policy must keep them separate.
    for (int i = 0; i < 6; i++) {
        display_damage_add_rect(&damage, 2 * i, 0, 1, 1);
    }
    for (int i = 0; i < 6; i++) {
        display_damage_add_rect(&damage, 150 + 2 * i, 150, 1, 1);
    }
    CHECK(damage.rects_len <= DISPLAY_DAMAGE_MAX_RECTS);
    for (int i = 0; i < 6; i++) {
        CHECK(damage_covers_pixel(&damage, 2 * i, 0));
        CHECK(damage_covers_pixel(&damage, 150 + 2 * i, 150));
    }
    CHECK(!damage_covers_pixel(&damage, 75, 75));
    CHECK(!damage_covers_pixel(&damage, 12, 12));
    check_invariants(&damage);
}

static void test_add_damage(void)
{
    struct DisplayDamage a;
    struct DisplayDamage b;
    display_damage_init(&a, 50, 50);
    display_damage_init(&b, 100, 100);

    display_damage_add_rect(&b, 10, 10, 10, 10);
    display_damage_add_rect(&b, 40, 40, 20, 20);
    display_damage_add_rect(&b, 70, 70, 10, 10);
    display_damage_add_damage(&a, &b);
    CHECK(a.rects_len == 2);
    CHECK(damage_covers_pixel(&a, 15, 15));
    CHECK(damage_covers_pixel(&a, 49, 49));
    CHECK(!damage_covers_pixel(&a, 30, 30));
    check_invariants(&a);
}

static void test_rows_span(void)
{
    struct DisplayDamage damage;
    display_damage_init(&damage, 64, 64);
    display_damage_add_rect(&damage, 0, 0, 4, 4);
    display_damage_add_rect(&damage, 10, 2, 5, 5);
    display_damage_add_rect(&damage, 30, 20, 2, 2);

    int x_start = -1;
    int x_end = -1;
    CHECK(display_damage_rows_span(&damage, 3, 4, &x_start, &x_end));
    CHECK(x_start == 0 && x_end == 15);
    CHECK(display_damage_rows_span(&damage, 5, 7, &x_start, &x_end));
    CHECK(x_start == 10 && x_end == 15);
    CHECK(!display_damage_rows_span(&damage, 8, 20, &x_start, &x_end));
    CHECK(display_damage_rows_span(&damage, 16, 24, &x_start, &x_end));
    CHECK(x_start == 30 && x_end == 32);
    CHECK(display_damage_rows_span(&damage, 0, 64, &x_start, &x_end));
    CHECK(x_start == 0 && x_end == 32);
}

// Whatever is added must stay covered, whatever was not added should stay
// uncovered unless merging had to grow a rectangle.
static void test_random_coverage(void)
{
    const int screen_w = 40;
    const int screen_h = 30;
    srand(1234);

    for (int iteration = 0; iteration < 2000; iteration++) {
        struct DisplayDamage damage;
        display_damage_init(&damage, screen_w, screen_h);

        static bool expected[30][40];
        for (int y = 0; y < screen_h; y++) {
            for (int x = 0; x < screen_w; x++) {
                expected[y][x] = false;
            }
        }

        int rects_count = rand() % 20;
        for (int i = 0; i < rects_count; i++) {
            int x = rand() % (screen_w + 20) - 10;
            int y = rand() % (screen_h + 20) - 10;
            int width = rand() % 25 - 2;
            int height = rand() % 20 - 2;
            display_damage_add_rect(&damage, x, y, width, height);

            for (int py = y; py < y + height; py++) {
                for (int px = x; px < x + width; px++) {
                    if (px >= 0 && px < screen_w && py >= 0 && py < screen_h) {
                        expected[py][px] = true;
                    }
                }
            }
        }

        check_invariants(&damage);
        for (int y = 0; y < screen_h; y++) {
            for (int x = 0; x < screen_w; x++) {
                if (expected[y][x]) {
                    CHECK(damage_covers_pixel(&damage, x, y));
                }
            }
        }
        if (rects_count <= DISPLAY_DAMAGE_MAX_RECTS) {
            // No merge could have happened: coverage must be exact.
            for (int y = 0; y < screen_h; y++) {
                for (int x = 0; x < screen_w; x++) {
                    CHECK(damage_covers_pixel(&damage, x, y) == expected[y][x]);
                }
            }
        }
        if (failures) {
            fprintf(stderr, "random coverage failed at iteration %i\n", iteration);
            return;
        }
    }
}

int main(void)
{
    test_empty_and_degenerate();
    test_clipping();
    test_containment();
    test_merging();
    test_add_damage();
    test_rows_span();
    test_random_coverage();

    if (failures) {
        fprintf(stderr, "%i check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    printf("display_damage: all tests passed\n");

    return EXIT_SUCCESS;
}
