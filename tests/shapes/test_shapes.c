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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "shape.h"

static int failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                #cond);                                                      \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static void check_render(struct ShapeData *shape, const char *expected, int line)
{
    int bx, by, bw, bh;
    shape_bounds(shape, &bx, &by, &bw, &bh);
    char *buf = malloc(bh * (bw + 1) + 1);
    char *p = buf;
    for (int y = by; y < by + bh; y++) {
        for (int x = bx; x < bx + bw; x++) {
            *p++ = shape_contains(shape, x, y) ? '#' : '.';
        }
        *p++ = '\n';
    }
    *p = '\0';
    if (strcmp(buf, expected) != 0) {
        fprintf(stderr, "%s:%d: render mismatch\nexpected:\n%sgot:\n%s", __FILE__, line,
            expected, buf);
        failures++;
    }
    free(buf);
}

#define CHECK_RENDER(shape, expected) check_render(shape, expected, __LINE__)

static void check_bounds(struct ShapeData *shape, int x, int y, int w, int h, int line)
{
    int bx, by, bw, bh;
    shape_bounds(shape, &bx, &by, &bw, &bh);
    if (bx != x || by != y || bw != w || bh != h) {
        fprintf(stderr, "%s:%d: bounds (%d,%d,%d,%d) != expected (%d,%d,%d,%d)\n", __FILE__,
            line, bx, by, bw, bh, x, y, w, h);
        failures++;
    }
}

#define CHECK_BOUNDS(shape, x, y, w, h) check_bounds(shape, x, y, w, h, __LINE__)

static bool same_pixels(struct ShapeData *a, struct ShapeData *b)
{
    int ax, ay, aw, ah;
    int bx, by, bw, bh;
    shape_bounds(a, &ax, &ay, &aw, &ah);
    shape_bounds(b, &bx, &by, &bw, &bh);
    int x0 = ax < bx ? ax : bx;
    int y0 = ay < by ? ay : by;
    int x1 = ax + aw > bx + bw ? ax + aw : bx + bw;
    int y1 = ay + ah > by + bh ? ay + ah : by + bh;
    for (int y = y0 - 1; y <= y1; y++) {
        for (int x = x0 - 1; x <= x1; x++) {
            if (shape_contains(a, x, y) != shape_contains(b, x, y)) {
                return false;
            }
        }
    }
    return true;
}

static void test_circle_r2(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_ellipse(10, 20, 2, 2)) != NULL);
    CHECK_BOUNDS(s, 8, 18, 5, 5);
    CHECK_RENDER(s,
        ".###.\n"
        "#####\n"
        "#####\n"
        "#####\n"
        ".###.\n");
    shape_destroy(s);
}

static void test_circle_small(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_ellipse(0, 0, 1, 1)) != NULL);
    CHECK_BOUNDS(s, -1, -1, 3, 3);
    CHECK_RENDER(s,
        ".#.\n"
        "###\n"
        ".#.\n");
    shape_destroy(s);

    CHECK((s = shape_new_ellipse(0, 0, 3, 3)) != NULL);
    CHECK_RENDER(s,
        "..###..\n"
        ".#####.\n"
        "#######\n"
        "#######\n"
        "#######\n"
        ".#####.\n"
        "..###..\n");
    shape_destroy(s);

    CHECK((s = shape_new_ellipse(0, 0, 3, 1)) != NULL);
    CHECK_RENDER(s,
        ".#####.\n"
        "#######\n"
        ".#####.\n");
    shape_destroy(s);
}

static void test_ellipse_symmetry(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_ellipse(0, 0, 9, 4)) != NULL);
    CHECK_BOUNDS(s, -9, -4, 19, 9);
    for (int y = -4; y <= 4; y++) {
        for (int x = -9; x <= 9; x++) {
            CHECK(shape_contains(s, x, y) == shape_contains(s, -x, y));
            CHECK(shape_contains(s, x, y) == shape_contains(s, x, -y));
        }
    }
    CHECK(shape_contains(s, 9, 0));
    CHECK(shape_contains(s, 0, 4));
    CHECK(!shape_contains(s, 10, 0));
    CHECK(!shape_contains(s, 9, 4));
    shape_destroy(s);
}

static void test_ellipse_invalid(void)
{
    CHECK(shape_new_ellipse(0, 0, 0, 5) == NULL);
    CHECK(shape_new_ellipse(0, 0, 5, -1) == NULL);
}

static void test_rounded_rect(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_rounded_rect(3, 4, 6, 4, 2)) != NULL);
    CHECK_BOUNDS(s, 3, 4, 6, 4);
    CHECK_RENDER(s,
        ".####.\n"
        "######\n"
        "######\n"
        ".####.\n");
    shape_destroy(s);
}

static void test_rounded_rect_small_radii(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_rounded_rect(0, 0, 5, 3, 1)) != NULL);
    CHECK_RENDER(s,
        ".###.\n"
        "#####\n"
        ".###.\n");
    shape_destroy(s);

    CHECK((s = shape_new_rounded_rect(0, 0, 7, 5, 1)) != NULL);
    CHECK_RENDER(s,
        ".#####.\n"
        "#######\n"
        "#######\n"
        "#######\n"
        ".#####.\n");
    shape_destroy(s);

    CHECK((s = shape_new_rounded_rect(0, 0, 7, 5, 2)) != NULL);
    CHECK_RENDER(s,
        ".#####.\n"
        "#######\n"
        "#######\n"
        "#######\n"
        ".#####.\n");
    shape_destroy(s);

    CHECK((s = shape_new_rounded_rect(0, 0, 9, 7, 3)) != NULL);
    CHECK_RENDER(s,
        "..#####..\n"
        ".#######.\n"
        "#########\n"
        "#########\n"
        "#########\n"
        ".#######.\n"
        "..#####..\n");
    shape_destroy(s);

    CHECK((s = shape_new_rounded_rect(0, 0, 2, 2, 1)) != NULL);
    CHECK_RENDER(s,
        "##\n"
        "##\n");
    shape_destroy(s);
}

static void test_rounded_rect_matches_circle(void)
{
    for (int r = 1; r <= 20; r++) {
        struct ShapeData *rr;
        struct ShapeData *circle;
        CHECK((rr = shape_new_rounded_rect(7 - r, -3 - r, 2 * r + 1, 2 * r + 1, r)) != NULL);
        CHECK((circle = shape_new_ellipse(7, -3, r, r)) != NULL);
        if (!same_pixels(rr, circle)) {
            fprintf(stderr, "%s:%d: rounded_rect radius %d differs from circle\n", __FILE__,
                __LINE__, r);
            failures++;
        }
        shape_destroy(rr);
        shape_destroy(circle);
    }
}

static void test_rounded_rect_radius_zero_is_rect(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_rounded_rect(0, 0, 3, 2, 0)) != NULL);
    CHECK_RENDER(s,
        "###\n"
        "###\n");
    shape_destroy(s);
}

static void test_rounded_rect_radius_clamped(void)
{
    struct ShapeData *a;
    struct ShapeData *b;
    CHECK((a = shape_new_rounded_rect(0, 0, 10, 6, 100)) != NULL);
    CHECK((b = shape_new_rounded_rect(0, 0, 10, 6, 2)) != NULL);
    CHECK(shape_equal(a, b));
    shape_destroy(b);
    CHECK((b = shape_new_rounded_rect(0, 0, 10, 6, 1)) != NULL);
    CHECK(!shape_equal(a, b));
    shape_destroy(a);
    shape_destroy(b);
}

static void test_rounded_rect_invalid(void)
{
    CHECK(shape_new_rounded_rect(0, 0, 0, 5, 1) == NULL);
    CHECK(shape_new_rounded_rect(0, 0, 5, -2, 1) == NULL);
    CHECK(shape_new_rounded_rect(0, 0, 5, 5, -1) == NULL);
}

static void test_negative_coordinates(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_ellipse(-10, -10, 2, 2)) != NULL);
    CHECK_RENDER(s,
        ".###.\n"
        "#####\n"
        "#####\n"
        "#####\n"
        ".###.\n");
    shape_destroy(s);

    CHECK((s = shape_new_rounded_rect(-3, -7, 6, 4, 2)) != NULL);
    CHECK_RENDER(s,
        ".####.\n"
        "######\n"
        "######\n"
        ".####.\n");
    shape_destroy(s);
}

static void test_equal(void)
{
    struct ShapeData *a;
    struct ShapeData *b;
    CHECK((a = shape_new_ellipse(1, 2, 3, 4)) != NULL);
    CHECK((b = shape_new_ellipse(1, 2, 3, 4)) != NULL);
    CHECK(shape_equal(a, b));
    shape_destroy(b);
    CHECK((b = shape_new_ellipse(1, 2, 3, 5)) != NULL);
    CHECK(!shape_equal(a, b));
    shape_destroy(b);
    CHECK((b = shape_new_rounded_rect(1, 2, 3, 4, 1)) != NULL);
    CHECK(!shape_equal(a, b));
    shape_destroy(a);
    shape_destroy(b);
}

static void test_large_values(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_ellipse(0, 0, 10000, 10000)) != NULL);
    CHECK(shape_contains(s, 10000, 0));
    CHECK(!shape_contains(s, 10000, 10000));
    shape_destroy(s);
}

static void check_runs(struct ShapeData *shape, int line)
{
    int bx, by, bw, bh;
    shape_bounds(shape, &bx, &by, &bw, &bh);
    int end = bx + bw;
    bool *row = malloc(sizeof(bool) * bw);
    int *expected = malloc(sizeof(int) * bw);

    for (int k = 0; k < 2 * bh; k++) {
        int y = (k < bh) ? by + k : by + 2 * bh - 1 - k;
        for (int i = 0; i < bw; i++) {
            row[i] = shape_contains(shape, bx + i, y);
        }
        for (int i = bw - 1; i >= 0; i--) {
            expected[i] = (i + 1 < bw && row[i + 1] == row[i]) ? expected[i + 1] + 1 : 1;
        }
        for (int x = bx; x < end; x++) {
            bool inside;
            int run = shape_run(shape, x, y, &inside);
            if (run < 1 || x + run > end) {
                fprintf(stderr, "%s:%d: bad run %d at (%d,%d)\n", __FILE__, line, run, x, y);
                failures++;
                goto out;
            }
            if (inside != row[x - bx] || run != expected[x - bx]) {
                fprintf(stderr, "%s:%d: run at (%d,%d) is %d/%s, expected %d/%s\n", __FILE__,
                    line, x, y, run, inside ? "in" : "out", expected[x - bx],
                    row[x - bx] ? "in" : "out");
                failures++;
                goto out;
            }
        }
    }
out:
    free(row);
    free(expected);
}

#define CHECK_RUNS(shape) check_runs(shape, __LINE__)

static void check_run_outside_bbox(struct ShapeData *s, int line)
{
    int bx, by, bw, bh;
    shape_bounds(s, &bx, &by, &bw, &bh);
    int xs[] = { INT_MIN, INT_MIN + 1, bx - 100000, bx - 1, bx, bx + bw - 1, bx + bw, INT_MAX };
    int ys[] = { INT_MIN, by - 1, by, by + bh - 1, by + bh, INT_MAX };
    for (size_t i = 0; i < sizeof(xs) / sizeof(xs[0]); i++) {
        for (size_t j = 0; j < sizeof(ys) / sizeof(ys[0]); j++) {
            int x = xs[i];
            int y = ys[j];
            bool in_rows = y >= by && y < by + bh;
            if (in_rows && x >= bx && x < bx + bw) {
                continue;
            }
            long long gap = (long long) bx - x;
            int expected = (in_rows && x < bx) ? (gap > INT_MAX ? INT_MAX : (int) gap) : 1;
            bool inside = true;
            int run = shape_run(s, x, y, &inside);
            if (inside || run != expected) {
                fprintf(stderr, "%s:%d: run at (%d,%d) is %d/%s, expected %d/out\n", __FILE__, line,
                    x, y, run, inside ? "in" : "out", expected);
                failures++;
            }
        }
    }
}

static void test_run_outside_bbox(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_rounded_rect(10, 20, 30, 40, 5)) != NULL);
    check_run_outside_bbox(s, __LINE__);
    shape_destroy(s);
    CHECK((s = shape_new_ellipse(-5, 7, 9, 4)) != NULL);
    check_run_outside_bbox(s, __LINE__);
    shape_destroy(s);
}

static void test_runs_match_contains(void)
{
    struct ShapeData *s;

    int ellipses[][4] = { { 0, 0, 9, 4 }, { 3, -2, 1, 7 }, { 10, 20, 2, 2 }, { -7, 5, 1, 1 },
        { 0, 0, 30, 30 } };
    for (size_t i = 0; i < sizeof(ellipses) / sizeof(ellipses[0]); i++) {
        CHECK((s = shape_new_ellipse(ellipses[i][0], ellipses[i][1], ellipses[i][2], ellipses[i][3])) != NULL);
        CHECK_RUNS(s);
        shape_destroy(s);
    }

    int rects[][5] = { { 3, 4, 6, 4, 2 }, { 0, 0, 17, 9, 4 }, { -5, -5, 3, 20, 10 },
        { 0, 0, 1, 1, 0 }, { 2, 2, 40, 12, 6 } };
    for (size_t i = 0; i < sizeof(rects) / sizeof(rects[0]); i++) {
        CHECK((s = shape_new_rounded_rect(rects[i][0], rects[i][1], rects[i][2], rects[i][3], rects[i][4])) != NULL);
        CHECK_RUNS(s);
        shape_destroy(s);
    }
}

static uint32_t rng_state = 12345;

static int rng_range(int lo, int hi)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return lo + (int) ((rng_state >> 8) % (uint32_t) (hi - lo + 1));
}

static bool is_symmetric(struct ShapeData *s)
{
    int bx, by, bw, bh;
    shape_bounds(s, &bx, &by, &bw, &bh);
    for (int y = by; y < by + bh; y++) {
        for (int x = bx; x < bx + bw; x++) {
            bool in = shape_contains(s, x, y);
            if (in != shape_contains(s, 2 * bx + bw - 1 - x, y)
                || in != shape_contains(s, x, 2 * by + bh - 1 - y)) {
                return false;
            }
        }
    }
    return true;
}

static void test_ellipse_runs_random(void)
{
    for (int i = 0; i < 300; i++) {
        struct ShapeData *s;
        CHECK((s = shape_new_ellipse(rng_range(-30, 30), rng_range(-30, 30), rng_range(1, 40), rng_range(1, 40))) != NULL);
        CHECK_RUNS(s);
        CHECK(is_symmetric(s));
        shape_destroy(s);
    }
}

static void test_rounded_rect_runs_random(void)
{
    for (int i = 0; i < 300; i++) {
        struct ShapeData *s;
        CHECK((s = shape_new_rounded_rect(rng_range(-30, 30), rng_range(-30, 30), rng_range(1, 60), rng_range(1, 60), rng_range(0, 35))) != NULL);
        CHECK_RUNS(s);
        CHECK(is_symmetric(s));
        shape_destroy(s);
    }
}

static bool check_row_extreme(struct ShapeData *s, int y, int wx0, int wx1)
{
    int bx, by, bw, bh;
    shape_bounds(s, &bx, &by, &bw, &bh);
    int end = bx + bw;
    for (int x = bx; x < end;) {
        bool inside;
        int run = shape_run(s, x, y, &inside);
        if (run < 1 || run > end - x || shape_contains(s, x, y) != inside
            || shape_contains(s, x + run - 1, y) != inside
            || (run < end - x && shape_contains(s, x + run, y) == inside)) {
            return false;
        }
        x += run;
    }
    wx0 = wx0 < bx ? bx : wx0;
    wx1 = wx1 > end ? end : wx1;
    bool row[300];
    int expected[300];
    int n = wx1 - wx0;
    if (n <= 0) {
        return true;
    }
    if (n > 300) {
        return false;
    }
    for (int i = 0; i < n; i++) {
        row[i] = shape_contains(s, wx0 + i, y);
    }
    for (int i = n - 1; i >= 0; i--) {
        expected[i] = (i + 1 < n && row[i + 1] == row[i]) ? expected[i + 1] + 1 : 1;
    }
    for (int i = 0; i < n; i++) {
        int x = wx0 + i;
        bool inside;
        int run = shape_run(s, x, y, &inside);
        if (run < 1 || run > end - x || inside != row[i]) {
            return false;
        }
        if (i + expected[i] < n) {
            if (run != expected[i]) {
                return false;
            }
        } else if (run < expected[i] || shape_contains(s, x + run - 1, y) != inside
            || (run < end - x && shape_contains(s, x + run, y) == inside)) {
            return false;
        }
    }
    return true;
}

static void check_extreme(struct ShapeData *s, const char *what, int line)
{
    int bx, by, bw, bh;
    shape_bounds(s, &bx, &by, &bw, &bh);
    bool ok = !shape_contains(s, bx - 1, by) && !shape_contains(s, bx + bw, by)
        && !shape_contains(s, bx, by - 1) && !shape_contains(s, bx, by + bh)
        && !shape_contains(s, INT_MIN, INT_MIN) && !shape_contains(s, INT_MAX, INT_MAX);
    int edge_rows[] = { by, by + 1, by + bh / 2, by + bh - 2, by + bh - 1 };
    for (size_t i = 0; ok && i < sizeof(edge_rows) / sizeof(edge_rows[0]); i++) {
        if (edge_rows[i] >= by && edge_rows[i] < by + bh) {
            ok = check_row_extreme(s, edge_rows[i], bx, bx + 4)
                && check_row_extreme(s, edge_rows[i], bx + bw - 4, bx + bw);
        }
    }
    int y0 = by > -2 ? by : -2;
    int y1 = by + bh < 242 ? by + bh : 242;
    for (int y = y0; ok && y < y1; y++) {
        ok = check_row_extreme(s, y, -2, 242);
    }
    if (!ok) {
        fprintf(stderr, "%s:%d: extreme %s at bbox (%d,%d,%d,%d) failed\n", __FILE__, line, what,
            bx, by, bw, bh);
        failures++;
    }
}

static void test_extreme_values(void)
{
    const int lim = SHAPE_VALUE_LIMIT;
    const int coords[] = { -lim, -lim / 2, 0, 120, lim };
    const int ncoords = sizeof(coords) / sizeof(coords[0]);
    const int sizes[] = { 1, 2, 3, lim / 2, lim - 1, lim };
    const int nsizes = sizeof(sizes) / sizeof(sizes[0]);
    struct ShapeData *s;

    for (int i = 0; i < ncoords; i++) {
        for (int j = 0; j < ncoords; j++) {
            int cx = coords[i];
            int cy = coords[j];
            for (int a = 0; a < nsizes; a++) {
                for (int b = 0; b < nsizes; b++) {
                    CHECK((s = shape_new_ellipse(cx, cy, sizes[a], sizes[b])) != NULL);
                    check_extreme(s, "ellipse", __LINE__);
                    shape_destroy(s);

                    CHECK((s = shape_new_rounded_rect(cx, cy, sizes[a], sizes[b], lim)) != NULL);
                    check_extreme(s, "rounded_rect", __LINE__);
                    shape_destroy(s);
                }
            }
        }
    }

}

static void test_out_of_range_rejected(void)
{
    const int over = SHAPE_VALUE_LIMIT + 1;
    const int under = -SHAPE_VALUE_LIMIT - 1;
    struct ShapeData *s;

    CHECK(shape_new_ellipse(over, 0, 1, 1) == NULL);
    CHECK(shape_new_ellipse(0, under, 1, 1) == NULL);
    CHECK(shape_new_ellipse(0, 0, over, 1) == NULL);
    CHECK(shape_new_ellipse(0, 0, 1, INT_MAX) == NULL);

    CHECK(shape_new_rounded_rect(under, 0, 1, 1, 0) == NULL);
    CHECK(shape_new_rounded_rect(0, over, 1, 1, 0) == NULL);
    CHECK(shape_new_rounded_rect(0, 0, over, 1, 0) == NULL);
    CHECK(shape_new_rounded_rect(0, 0, 1, over, 0) == NULL);
    CHECK(shape_new_rounded_rect(0, 0, 1, 1, over) == NULL);

    CHECK((s = shape_new_ellipse(SHAPE_VALUE_LIMIT, -SHAPE_VALUE_LIMIT, SHAPE_VALUE_LIMIT, 1)) != NULL);
    shape_destroy(s);
}

static void test_run_ellipse_values(void)
{
    struct ShapeData *s;
    bool inside;
    CHECK((s = shape_new_ellipse(10, 20, 2, 2)) != NULL);
    CHECK(shape_run(s, 8, 18, &inside) == 1 && !inside);
    CHECK(shape_run(s, 9, 18, &inside) == 3 && inside);
    CHECK(shape_run(s, 10, 18, &inside) == 2 && inside);
    CHECK(shape_run(s, 12, 18, &inside) == 1 && !inside);
    CHECK(shape_run(s, 8, 20, &inside) == 5 && inside);
    shape_destroy(s);
}

int main(void)
{
    test_circle_r2();
    test_circle_small();
    test_ellipse_symmetry();
    test_ellipse_invalid();
    test_rounded_rect();
    test_rounded_rect_small_radii();
    test_rounded_rect_matches_circle();
    test_rounded_rect_radius_zero_is_rect();
    test_rounded_rect_radius_clamped();
    test_rounded_rect_invalid();
    test_negative_coordinates();
    test_equal();
    test_large_values();
    test_runs_match_contains();
    test_run_outside_bbox();
    test_ellipse_runs_random();
    test_rounded_rect_runs_random();
    test_run_ellipse_values();
    test_out_of_range_rejected();
    test_extreme_values();

    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    printf("all shape tests passed\n");
    return 0;
}
