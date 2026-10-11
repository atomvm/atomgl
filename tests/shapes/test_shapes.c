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

static void test_line_horizontal_thickness(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_line(0, 0, 4, 0, 1)) != NULL);
    CHECK_BOUNDS(s, 0, 0, 5, 1);
    CHECK_RENDER(s, "#####\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(0, 0, 4, 0, 2)) != NULL);
    CHECK_BOUNDS(s, 0, -1, 5, 2);
    CHECK_RENDER(s,
        "#####\n"
        "#####\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(0, 0, 4, 0, 3)) != NULL);
    CHECK_BOUNDS(s, -1, -1, 7, 3);
    CHECK_RENDER(s,
        ".#####.\n"
        "#######\n"
        ".#####.\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(0, 0, 0, 2, 2)) != NULL);
    CHECK_BOUNDS(s, -1, 0, 2, 3);
    CHECK_RENDER(s,
        "##\n"
        "##\n"
        "##\n");
    shape_destroy(s);
}

static void test_line_even_caps(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_line(0, 0, 4, 0, 4)) != NULL);
    CHECK_BOUNDS(s, -1, -2, 7, 4);
    CHECK_RENDER(s,
        "#######\n"
        "#######\n"
        "#######\n"
        "#######\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(0, 0, 4, 0, 6)) != NULL);
    CHECK_BOUNDS(s, -2, -3, 9, 6);
    CHECK_RENDER(s,
        ".#######.\n"
        "#########\n"
        "#########\n"
        "#########\n"
        "#########\n"
        ".#######.\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(0, 0, 0, 4, 4)) != NULL);
    CHECK_BOUNDS(s, -2, -1, 4, 7);
    CHECK_RENDER(s,
        "####\n"
        "####\n"
        "####\n"
        "####\n"
        "####\n"
        "####\n"
        "####\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(0, 4, 0, 0, 6)) != NULL);
    CHECK_BOUNDS(s, -3, -2, 6, 9);
    CHECK_RENDER(s,
        ".####.\n"
        "######\n"
        "######\n"
        "######\n"
        "######\n"
        "######\n"
        "######\n"
        "######\n"
        ".####.\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(0, 0, 5, 5, 2)) != NULL);
    CHECK_BOUNDS(s, 0, -1, 6, 7);
    CHECK_RENDER(s,
        "#.....\n"
        "##....\n"
        ".##...\n"
        "..##..\n"
        "...##.\n"
        "....##\n"
        ".....#\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(0, 0, 5, 5, 4)) != NULL);
    CHECK_BOUNDS(s, -1, -2, 8, 9);
    CHECK_RENDER(s,
        "###.....\n"
        "###.....\n"
        "####....\n"
        "#####...\n"
        "..####..\n"
        "...#####\n"
        "....####\n"
        ".....###\n"
        ".....###\n");
    shape_destroy(s);

    CHECK((s = shape_new_line(5, 5, 0, 0, 6)) != NULL);
    CHECK_BOUNDS(s, -2, -3, 10, 11);
    CHECK_RENDER(s,
        ".###......\n"
        "#####.....\n"
        "#####.....\n"
        "######....\n"
        "#######...\n"
        ".########.\n"
        "...#######\n"
        "....######\n"
        ".....#####\n"
        ".....#####\n"
        "......###.\n");
    shape_destroy(s);
}

static void test_line_diagonal_connected(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_line(0, 0, 10, 10, 1)) != NULL);
    CHECK_BOUNDS(s, 0, 0, 11, 11);
    for (int y = 0; y <= 10; y++) {
        for (int x = 0; x <= 10; x++) {
            CHECK(shape_contains(s, x, y) == (x == y));
        }
    }
    shape_destroy(s);
}

static void test_line_shallow_one_pixel_per_column(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_line(0, 0, 20, 10, 1)) != NULL);
    CHECK_BOUNDS(s, 0, 0, 21, 11);
    int count = 0;
    for (int y = 0; y <= 10; y++) {
        for (int x = 0; x <= 20; x++) {
            bool in = shape_contains(s, x, y);
            CHECK(in == (y == x / 2));
            count += in ? 1 : 0;
        }
    }
    CHECK(count == 21);
    shape_destroy(s);
}

static void test_line_thick_sloped(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_line(0, 0, 12, 5, 3)) != NULL);
    CHECK_BOUNDS(s, -1, -1, 15, 8);
    CHECK_RENDER(s,
        ".##............\n"
        "#####..........\n"
        ".#######.......\n"
        "...#######.....\n"
        ".....#######...\n"
        "........######.\n"
        "..........#####\n"
        "............##.\n");
    shape_destroy(s);
}

static void test_line_reversed_is_same(void)
{
    struct ShapeData *a;
    struct ShapeData *b;
    CHECK((a = shape_new_line(-3, 7, 12, -4, 4)) != NULL);
    CHECK((b = shape_new_line(12, -4, -3, 7, 4)) != NULL);
    CHECK(same_pixels(a, b));
    shape_destroy(a);
    shape_destroy(b);
}

static void test_line_zero_length_is_dot(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_line(5, 5, 5, 5, 1)) != NULL);
    CHECK_BOUNDS(s, 5, 5, 1, 1);
    CHECK(shape_contains(s, 5, 5));
    CHECK(!shape_contains(s, 6, 5));
    shape_destroy(s);

    CHECK((s = shape_new_line(5, 5, 5, 5, 2)) != NULL);
    CHECK_BOUNDS(s, 5, 5, 1, 1);
    CHECK_RENDER(s, "#\n");
    shape_destroy(s);

    struct ShapeData *disc;
    CHECK((s = shape_new_line(5, 5, 5, 5, 5)) != NULL);
    CHECK((disc = shape_new_ellipse(5, 5, 2, 2)) != NULL);
    CHECK_BOUNDS(s, 3, 3, 5, 5);
    CHECK(same_pixels(s, disc));
    shape_destroy(s);
    shape_destroy(disc);
}

static void test_line_invalid(void)
{
    CHECK(shape_new_line(0, 0, 4, 4, 0) == NULL);
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
    CHECK((b = shape_new_line(1, 2, 3, 4, 1)) != NULL);
    CHECK(!shape_equal(a, b));
    shape_destroy(a);
    shape_destroy(b);
}

static void test_arc_quarter(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_arc(0, 0, 10, 3, 0, 90)) != NULL);
    CHECK_BOUNDS(s, -10, -10, 21, 21);
    CHECK(shape_contains(s, 7, 7));
    CHECK(!shape_contains(s, 7, -7));
    CHECK(!shape_contains(s, -7, 7));
    CHECK(!shape_contains(s, 0, 0));
    CHECK(!shape_contains(s, 3, 3));
    CHECK(shape_contains(s, 10, 0));
    CHECK(shape_contains(s, 0, 10));
    shape_destroy(s);
}

static void test_arc_full_ring(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_arc(0, 0, 10, 2, 0, 360)) != NULL);
    CHECK(shape_contains(s, 9, 0));
    CHECK(shape_contains(s, -9, 0));
    CHECK(shape_contains(s, 0, 9));
    CHECK(shape_contains(s, 0, -9));
    CHECK(!shape_contains(s, 0, 0));
    shape_destroy(s);
}

static void test_arc_over_180(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_arc(0, 0, 10, 3, 0, 270)) != NULL);
    CHECK(shape_contains(s, 7, 7));
    CHECK(shape_contains(s, -7, 7));
    CHECK(shape_contains(s, -7, -7));
    CHECK(!shape_contains(s, 7, -7));
    shape_destroy(s);
}

static void test_arc_wraparound(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_arc(0, 0, 10, 3, 300, 60)) != NULL);
    CHECK(shape_contains(s, 9, 0));
    CHECK(shape_contains(s, 5, -8));
    CHECK(shape_contains(s, 5, 8));
    CHECK(!shape_contains(s, 0, 9));
    CHECK(!shape_contains(s, -9, 0));
    shape_destroy(s);
}

static void test_arc_negative_angles(void)
{
    struct ShapeData *a;
    struct ShapeData *b;
    CHECK((a = shape_new_arc(0, 0, 10, 3, -90, 0)) != NULL);
    CHECK((b = shape_new_arc(0, 0, 10, 3, 270, 360)) != NULL);
    for (int y = -10; y <= 10; y++) {
        for (int x = -10; x <= 10; x++) {
            CHECK(shape_contains(a, x, y) == shape_contains(b, x, y));
        }
    }
    CHECK(shape_contains(a, 7, -7));
    CHECK(!shape_contains(a, 7, 7));
    shape_destroy(a);
    shape_destroy(b);
}

static void check_arc_same(int start_deg, int end_deg, int ref_start, int ref_end, int line)
{
    struct ShapeData *a = shape_new_arc(0, 0, 12, 4, start_deg, end_deg);
    struct ShapeData *b = shape_new_arc(0, 0, 12, 4, ref_start, ref_end);
    if (a == NULL || b == NULL) {
        fprintf(stderr, "%s:%d: arc init failed\n", __FILE__, line);
        failures++;
        shape_destroy(a);
        shape_destroy(b);
        return;
    }
    if (!same_pixels(a, b)) {
        fprintf(stderr, "%s:%d: arc (%d, %d) differs from (%d, %d)\n", __FILE__, line,
            start_deg, end_deg, ref_start, ref_end);
        failures++;
    }
    shape_destroy(a);
    shape_destroy(b);
}

#define CHECK_ARC_SAME(s, e, rs, re) check_arc_same(s, e, rs, re, __LINE__)

static int arc_sweep(int start_deg, int end_deg)
{
    struct ShapeData *s = shape_new_arc(0, 0, 10, 2, start_deg, end_deg);
    if (s == NULL) {
        return -1;
    }
    int start = ((start_deg % 360) + 360) % 360;
    int sweep = 0;
    for (int k = 1; k <= 360 && sweep == 0; k++) {
        struct ShapeData *ref = shape_new_arc(0, 0, 10, 2, start, start + k);
        if (shape_equal(s, ref)) {
            sweep = k;
        }
        shape_destroy(ref);
    }
    shape_destroy(s);
    return sweep;
}

static void test_arc_sweep_rule(void)
{
    CHECK(arc_sweep(0, 90) == 90);
    CHECK(arc_sweep(0, -90) == 270);
    CHECK(arc_sweep(0, -359) == 1);
    CHECK(arc_sweep(0, 360) == 360);
    CHECK(arc_sweep(0, -360) == 360);
    CHECK(arc_sweep(0, 720) == 360);
    CHECK(arc_sweep(0, 400) == 40);
    CHECK(arc_sweep(90, 0) == 270);
    CHECK(arc_sweep(-90, 0) == 90);
    CHECK(arc_sweep(0, 0) == -1);
    CHECK(arc_sweep(45, 45) == -1);
    CHECK(arc_sweep(-360, -360) == -1);
    CHECK(arc_sweep(INT_MIN, INT_MAX) == 255);
    CHECK(arc_sweep(INT_MAX, INT_MIN) == 105);

    CHECK_ARC_SAME(0, -90, 0, 270);
    CHECK_ARC_SAME(0, -359, 0, 1);
    CHECK_ARC_SAME(0, 400, 0, 40);
    CHECK_ARC_SAME(0, -360, 0, 360);
    CHECK_ARC_SAME(0, 720, 0, 360);
    CHECK_ARC_SAME(90, 0, 90, 360);
    CHECK_ARC_SAME(-90, 0, 270, 360);

    struct ShapeData *s;
    CHECK((s = shape_new_arc(0, 0, 12, 4, 0, -360)) != NULL);
    CHECK(shape_contains(s, 10, 0));
    CHECK(shape_contains(s, 0, 10));
    CHECK(shape_contains(s, -10, 0));
    CHECK(shape_contains(s, 0, -10));
    CHECK(shape_contains(s, 7, -7));
    shape_destroy(s);

    CHECK((s = shape_new_arc(0, 0, 40, 4, 0, -359)) != NULL);
    CHECK(shape_contains(s, 38, 0));
    CHECK(!shape_contains(s, 0, 38));
    CHECK(!shape_contains(s, -38, 0));
    CHECK(!shape_contains(s, 27, -27));
    shape_destroy(s);
}

static void test_arc_pie(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_arc(0, 0, 10, 10, 0, 90)) != NULL);
    CHECK(shape_contains(s, 0, 0));
    CHECK(shape_contains(s, 2, 2));
    CHECK(!shape_contains(s, -2, -2));
    shape_destroy(s);
}

static void test_arc_matches_circle_when_full_pie(void)
{
    struct ShapeData *arc;
    struct ShapeData *circle;
    CHECK((arc = shape_new_arc(3, 4, 6, 100, 0, 360)) != NULL);
    CHECK((circle = shape_new_ellipse(3, 4, 6, 6)) != NULL);
    for (int y = -3; y <= 11; y++) {
        for (int x = -4; x <= 10; x++) {
            CHECK(shape_contains(arc, x, y) == shape_contains(circle, x, y));
        }
    }
    shape_destroy(arc);
    shape_destroy(circle);
}

static void test_arc_ring_matches_circles(void)
{
    for (int r = 1; r <= 15; r++) {
        for (int t = 1; t <= r + 1; t++) {
            struct ShapeData *arc;
            struct ShapeData *outer;
            struct ShapeData *hole;
            CHECK((arc = shape_new_arc(2, 3, r, t, 0, 360)) != NULL);
            CHECK((outer = shape_new_ellipse(2, 3, r, r)) != NULL);
            hole = r - t > 0 ? shape_new_ellipse(2, 3, r - t, r - t) : NULL;
            bool have_hole = hole != NULL;
            for (int y = 3 - r - 1; y <= 3 + r + 1; y++) {
                for (int x = 2 - r - 1; x <= 2 + r + 1; x++) {
                    bool expected = shape_contains(outer, x, y)
                        && !(have_hole && shape_contains(hole, x, y));
                    if (shape_contains(arc, x, y) != expected) {
                        fprintf(stderr, "%s:%d: ring r=%d t=%d differs at (%d,%d)\n", __FILE__,
                            __LINE__, r, t, x, y);
                        failures++;
                    }
                }
            }
            shape_destroy(arc);
            shape_destroy(outer);
            shape_destroy(hole);
        }
    }
}

static void check_runs(struct ShapeData *shape, int line);

static bool is_empty(struct ShapeData *s)
{
    int bx, by, bw, bh;
    shape_bounds(s, &bx, &by, &bw, &bh);
    for (int y = by; y < by + bh; y++) {
        for (int x = bx; x < bx + bw; x++) {
            if (shape_contains(s, x, y)) {
                return false;
            }
        }
    }
    return true;
}

static void test_arc_tiny_sweep_visible(void)
{
    static const int radii[] = { 5, 10, 20, 60 };
    static const int thicknesses[] = { 1, 3 };
    for (size_t i = 0; i < sizeof(radii) / sizeof(radii[0]); i++) {
        for (size_t j = 0; j < sizeof(thicknesses) / sizeof(thicknesses[0]); j++) {
            int empty = 0;
            for (int start = 0; start < 360; start++) {
                struct ShapeData *s = shape_new_arc(3, -2, radii[i], thicknesses[j], start, start + 1);
                CHECK(s != NULL);
                empty += is_empty(s) ? 1 : 0;
                if (radii[i] < 60 || start % 7 == 0) {
                    check_runs(s, __LINE__);
                }
                shape_destroy(s);
            }
            if (empty) {
                fprintf(stderr, "%s:%d: r=%d t=%d: %d of 360 one-degree arcs are empty\n", __FILE__,
                    __LINE__, radii[i], thicknesses[j], empty);
                failures++;
            }
        }
    }
}

static void test_arc_invalid(void)
{
    CHECK(shape_new_arc(0, 0, 0, 1, 0, 90) == NULL);
    CHECK(shape_new_arc(0, 0, 10, 0, 0, 90) == NULL);
    CHECK(shape_new_arc(0, 0, 10, 2, 45, 45) == NULL);
}

static void test_large_values(void)
{
    struct ShapeData *s;
    CHECK((s = shape_new_ellipse(0, 0, 10000, 10000)) != NULL);
    CHECK(shape_contains(s, 10000, 0));
    CHECK(!shape_contains(s, 10000, 10000));
    shape_destroy(s);
    CHECK((s = shape_new_arc(0, 0, 10000, 5, 0, 90)) != NULL);
    CHECK(shape_contains(s, 7071, 7071));
    CHECK(!shape_contains(s, 7071, -7071));
    shape_destroy(s);
    CHECK((s = shape_new_line(-2000, -2000, 2000, 2000, 3)) != NULL);
    CHECK(shape_contains(s, 1234, 1234));
    CHECK(!shape_contains(s, 1234, -1234));
    shape_destroy(s);
}

static void test_polygon_square_matches_rect(void)
{
    struct ShapePoint pts[] = { { 0, 0 }, { 4, 0 }, { 4, 4 }, { 0, 4 } };
    struct ShapeData *s;
    CHECK((s = shape_new_polygon(pts, 4)) != NULL);
    CHECK_BOUNDS(s, 0, 0, 4, 4);
    CHECK_RENDER(s,
        "####\n"
        "####\n"
        "####\n"
        "####\n");
    shape_destroy(s);
}

static void test_polygon_concave(void)
{
    struct ShapePoint pts[] = { { 0, 0 }, { 4, 0 }, { 4, 2 }, { 2, 2 }, { 2, 4 }, { 0, 4 } };
    struct ShapeData *s;
    CHECK((s = shape_new_polygon(pts, 6)) != NULL);
    CHECK_RENDER(s,
        "####\n"
        "####\n"
        "##..\n"
        "##..\n");
    shape_destroy(s);
}

static void test_polygon_even_odd_hole(void)
{
    struct ShapePoint pts[] = {
        { 0, 0 }, { 6, 0 }, { 6, 6 }, { 0, 6 }, { 0, 0 },
        { 2, 2 }, { 2, 4 }, { 4, 4 }, { 4, 2 }, { 2, 2 }
    };
    struct ShapeData *s;
    CHECK((s = shape_new_polygon(pts, 10)) != NULL);
    CHECK(shape_contains(s, 1, 4));
    CHECK(!shape_contains(s, 3, 3));
    CHECK(shape_contains(s, 5, 3));
    CHECK(shape_contains(s, 5, 5));
    shape_destroy(s);
}

static void test_polygon_triangle_rows_out_of_order(void)
{
    struct ShapePoint pts[] = { { 0, 0 }, { 8, 0 }, { 0, 8 } };
    struct ShapeData *s;
    CHECK((s = shape_new_polygon(pts, 3)) != NULL);
    CHECK(shape_contains(s, 0, 7));
    CHECK(shape_contains(s, 6, 0));
    CHECK(!shape_contains(s, 7, 7));
    CHECK(shape_contains(s, 0, 7));
    CHECK(!shape_contains(s, 1, 7));
    shape_destroy(s);
}

static void test_polygon_copies_points(void)
{
    struct ShapePoint pts[] = { { 0, 0 }, { 4, 0 }, { 4, 4 }, { 0, 4 } };
    struct ShapeData *s;
    CHECK((s = shape_new_polygon(pts, 4)) != NULL);
    pts[1].x = 100;
    CHECK(!shape_contains(s, 50, 1));
    CHECK(shape_contains(s, 3, 1));
    shape_destroy(s);
}

static void test_polygon_invalid(void)
{
    struct ShapePoint pts[] = { { 0, 0 }, { 4, 0 } };
    CHECK(shape_new_polygon(pts, 2) == NULL);
    CHECK(shape_new_polygon(NULL, 0) == NULL);
    CHECK(shape_polygon_begin(2) == NULL);
}

static void test_polygon_max_points(void)
{
    int n = SHAPE_POLYGON_MAX_POINTS + 1;
    struct ShapePoint *pts = malloc(sizeof(struct ShapePoint) * n);
    for (int i = 0; i < n; i++) {
        pts[i].x = i % 2;
        pts[i].y = i;
    }
    struct ShapeData *s;
    CHECK(shape_new_polygon(pts, n) == NULL);
    CHECK((s = shape_new_polygon(pts, n - 1)) != NULL);
    shape_destroy(s);

    CHECK(shape_polygon_begin(n) == NULL);
    CHECK((s = shape_polygon_begin(4)) != NULL);
    CHECK(shape_polygon_add_point(s, 0, 0));
    CHECK(shape_polygon_add_point(s, 4, 0));
    CHECK(shape_polygon_add_point(s, 4, 4));
    CHECK(!shape_polygon_end(s));
    CHECK(!shape_polygon_add_point(s, SHAPE_VALUE_LIMIT + 1, 4));
    CHECK(shape_polygon_add_point(s, 0, 4));
    CHECK(!shape_polygon_add_point(s, 0, 0));
    CHECK(shape_polygon_end(s));
    CHECK(shape_contains(s, 3, 3));
    shape_destroy(s);
    free(pts);
}

static void test_polygon_equal(void)
{
    struct ShapePoint p1[] = { { 0, 0 }, { 4, 0 }, { 0, 4 } };
    struct ShapePoint p2[] = { { 0, 0 }, { 4, 0 }, { 0, 5 } };
    struct ShapeData *a;
    struct ShapeData *b;
    struct ShapeData *c;
    CHECK((a = shape_new_polygon(p1, 3)) != NULL);
    CHECK((b = shape_new_polygon(p1, 3)) != NULL);
    CHECK((c = shape_new_polygon(p2, 3)) != NULL);
    shape_contains(a, 1, 1);
    CHECK(shape_equal(a, b));
    CHECK(!shape_equal(a, c));
    shape_destroy(a);
    shape_destroy(b);
    shape_destroy(c);
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

static long render_by_runs(struct ShapeData *s)
{
    int bx, by, bw, bh;
    shape_bounds(s, &bx, &by, &bw, &bh);
    long count = 0;
    for (int y = by; y < by + bh; y++) {
        for (int x = bx; x < bx + bw; x++) {
            bool inside;
            shape_run(s, x, y, &inside);
            count += inside ? 1 : 0;
        }
    }
    return count;
}

static void test_polygon_comb(void)
{
    int n = SHAPE_POLYGON_MAX_POINTS;
    struct ShapePoint *pts = malloc(sizeof(struct ShapePoint) * n);
    struct ShapeData *s;

    for (int i = 0; i < n; i++) {
        int tooth = i / 3;
        pts[i].x = tooth;
        pts[i].y = (i % 3 == 1) ? 240 : 0;
    }
    CHECK((s = shape_new_polygon(pts, n)) != NULL);
    CHECK(render_by_runs(s) == 0);
    CHECK_RUNS(s);
    shape_destroy(s);

    for (int t = 0; t < n / 4; t++) {
        pts[4 * t + 0] = (struct ShapePoint){ 2 * t, 240 };
        pts[4 * t + 1] = (struct ShapePoint){ 2 * t, 10 };
        pts[4 * t + 2] = (struct ShapePoint){ 2 * t + 1, 10 };
        pts[4 * t + 3] = (struct ShapePoint){ 2 * t + 1, 240 };
    }
    CHECK((s = shape_new_polygon(pts, n)) != NULL);
    CHECK(render_by_runs(s) == (long) (n / 4) * 230);
    CHECK_RUNS(s);
    shape_destroy(s);
    free(pts);
}

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
    CHECK((s = shape_new_line(0, 0, 50, 13, 5)) != NULL);
    check_run_outside_bbox(s, __LINE__);
    shape_destroy(s);
    CHECK((s = shape_new_arc(100, 100, 30, 8, 10, 250)) != NULL);
    check_run_outside_bbox(s, __LINE__);
    shape_destroy(s);
    CHECK((s = shape_new_arc(-SHAPE_VALUE_LIMIT, SHAPE_VALUE_LIMIT, SHAPE_VALUE_LIMIT, 1, 0, 360)) != NULL);
    check_run_outside_bbox(s, __LINE__);
    shape_destroy(s);
    struct ShapePoint pts[] = { { 0, 0 }, { 40, 10 }, { 10, 40 }, { 20, 15 } };
    CHECK((s = shape_new_polygon(pts, 4)) != NULL);
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

    int lines[][5] = { { 0, 0, 4, 0, 1 }, { 0, 0, 4, 0, 3 }, { 0, 0, 9, 3, 1 }, { 0, 0, 3, 9, 2 },
        { 9, 0, 0, 5, 4 }, { 5, 5, 5, 5, 1 }, { 0, 0, 0, 8, 3 }, { -3, 7, 12, -4, 5 },
        { 0, 0, 40, 1, 1 }, { 0, 0, 1, 40, 1 }, { 20, 3, -20, 9, 2 } };
    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        CHECK((s = shape_new_line(lines[i][0], lines[i][1], lines[i][2], lines[i][3], lines[i][4])) != NULL);
        CHECK_RUNS(s);
        shape_destroy(s);
    }

    int arcs[][6] = { { 0, 0, 10, 3, 0, 90 }, { 0, 0, 10, 3, 300, 60 }, { 0, 0, 10, 10, 0, 270 },
        { 0, 0, 6, 2, 0, 360 }, { 5, 5, 12, 4, -90, 180 }, { 0, 0, 1, 1, 0, 90 },
        { 0, 0, 20, 1, 45, 46 }, { 0, 0, 20, 5, 89, 91 }, { 0, 0, 20, 5, 179, 181 },
        { 0, 0, 20, 25, 10, 350 }, { 0, 0, 20, 5, 0, 180 }, { 0, 0, 20, 5, 90, 270 } };
    for (size_t i = 0; i < sizeof(arcs) / sizeof(arcs[0]); i++) {
        CHECK((s = shape_new_arc(arcs[i][0], arcs[i][1], arcs[i][2], arcs[i][3], arcs[i][4], arcs[i][5])) != NULL);
        CHECK_RUNS(s);
        shape_destroy(s);
    }

    struct ShapePoint square[] = { { 0, 0 }, { 4, 0 }, { 4, 4 }, { 0, 4 } };
    struct ShapePoint ell[] = { { 0, 0 }, { 4, 0 }, { 4, 2 }, { 2, 2 }, { 2, 4 }, { 0, 4 } };
    struct ShapePoint hole[] = { { 0, 0 }, { 6, 0 }, { 6, 6 }, { 0, 6 }, { 0, 0 }, { 2, 2 },
        { 2, 4 }, { 4, 4 }, { 4, 2 }, { 2, 2 } };
    struct ShapePoint tri[] = { { 0, 0 }, { 8, 0 }, { 0, 8 } };
    struct ShapePoint star[] = { { 10, 0 }, { 16, 19 }, { 0, 7 }, { 20, 7 }, { 4, 19 } };
    struct ShapePoint neg[] = { { -9, -3 }, { 5, -8 }, { 2, 6 } };
    CHECK((s = shape_new_polygon(square, 4)) != NULL);
    CHECK_RUNS(s);
    shape_destroy(s);
    CHECK((s = shape_new_polygon(ell, 6)) != NULL);
    CHECK_RUNS(s);
    shape_destroy(s);
    CHECK((s = shape_new_polygon(hole, 10)) != NULL);
    CHECK_RUNS(s);
    shape_destroy(s);
    CHECK((s = shape_new_polygon(tri, 3)) != NULL);
    CHECK_RUNS(s);
    shape_destroy(s);
    CHECK((s = shape_new_polygon(star, 5)) != NULL);
    CHECK_RUNS(s);
    shape_destroy(s);
    CHECK((s = shape_new_polygon(neg, 3)) != NULL);
    CHECK_RUNS(s);
    shape_destroy(s);
}

static uint32_t rng_state = 12345;

static int rng_range(int lo, int hi)
{
    rng_state = rng_state * 1103515245u + 12345u;
    return lo + (int) ((rng_state >> 8) % (uint32_t) (hi - lo + 1));
}

static void test_arc_runs_random(void)
{
    for (int i = 0; i < 400; i++) {
        int r = rng_range(1, 40);
        int t = rng_range(1, r + 3);
        int start = rng_range(-400, 400);
        int end = start + rng_range(-420, 420);
        int cx = rng_range(-20, 20);
        int cy = rng_range(-20, 20);
        struct ShapeData *s = shape_new_arc(cx, cy, r, t, start, end);
        if (s == NULL) {
            continue;
        }
        CHECK_RUNS(s);
        shape_destroy(s);
    }
}

static void test_polygon_runs_random(void)
{
    struct ShapePoint pts[40];
    for (int i = 0; i < 400; i++) {
        int n = rng_range(3, 40);
        int span = rng_range(1, 30);
        for (int j = 0; j < n; j++) {
            if (j > 0 && rng_range(0, 5) == 0) {
                pts[j] = pts[j - 1];
                pts[j].x += rng_range(-1, 1) * rng_range(0, span);
            } else {
                pts[j].x = rng_range(-span, span);
                pts[j].y = rng_range(-span, span);
            }
        }
        struct ShapeData *s = shape_new_polygon(pts, n);
        CHECK(s != NULL);
        CHECK_RUNS(s);
        shape_destroy(s);
    }
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

static bool ref_line_contains(int x1, int y1, int x2, int y2, int t, int x, int y)
{
    if (x1 == x2 && y1 == y2) {
        long long r = (t - 1) / 2;
        long long dx = x - x1;
        long long dy = y - y1;
        return (dx == 0 && dy == 0) || dx * dx + dy * dy < r * r + r;
    }
    bool x_major = llabs((long long) x2 - x1) >= llabs((long long) y2 - y1);
    long long ea[2] = { x_major ? x1 : y1, x_major ? x2 : y2 };
    long long eb[2] = { x_major ? y1 : x1, x_major ? y2 : x2 };
    for (int i = 0; i < 2; i++) {
        long long u = 2 * ((x_major ? x : y) - ea[i]);
        long long v = 2 * ((x_major ? y : x) - eb[i]) + (t % 2 == 0 ? 1 : 0);
        if (u * u + v * v < (long long) t * t - 1) {
            return true;
        }
    }
    long long a1 = x_major ? x1 : y1;
    long long b1 = x_major ? y1 : x1;
    long long a2 = x_major ? x2 : y2;
    long long b2 = x_major ? y2 : x2;
    long long a = x_major ? x : y;
    long long b = x_major ? y : x;
    if (a1 == a2 || a < (a1 < a2 ? a1 : a2) || a > (a1 < a2 ? a2 : a1)) {
        return false;
    }
    long long da = a2 - a1;
    long long o2 = 2 * ((b - b1) * da - (a - a1) * (b2 - b1));
    if (da < 0) {
        da = -da;
        o2 = -o2;
    }
    return -t * da <= o2 && o2 < t * da;
}

static bool line_matches_reference(struct ShapeData *s, int x1, int y1, int x2, int y2, int t)
{
    int margin = t + 2;
    int lo_x = (x1 < x2 ? x1 : x2) - margin;
    int hi_x = (x1 < x2 ? x2 : x1) + margin;
    int lo_y = (y1 < y2 ? y1 : y2) - margin;
    int hi_y = (y1 < y2 ? y2 : y1) + margin;
    for (int y = lo_y; y <= hi_y; y++) {
        for (int x = lo_x; x <= hi_x; x++) {
            if (shape_contains(s, x, y) != ref_line_contains(x1, y1, x2, y2, t, x, y)) {
                fprintf(stderr, "line (%d,%d)-(%d,%d) T=%d differs from the reference at (%d,%d)\n",
                    x1, y1, x2, y2, t, x, y);
                return false;
            }
        }
    }
    return true;
}

static void check_line_properties(int x1, int y1, int x2, int y2, int t)
{
    struct ShapeData *s;
    CHECK((s = shape_new_line(x1, y1, x2, y2, t)) != NULL);
    int bx, by, bw, bh;
    shape_bounds(s, &bx, &by, &bw, &bh);
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    bool x_major = dx >= dy;
    int a_lo = x_major ? bx : by;
    int a_hi = x_major ? bx + bw : by + bh;
    int b_lo = x_major ? by : bx;
    int b_hi = x_major ? by + bh : bx + bw;
    int a1 = x_major ? (x1 < x2 ? x1 : x2) : (y1 < y2 ? y1 : y2);
    int a2 = x_major ? (x1 < x2 ? x2 : x1) : (y1 < y2 ? y2 : y1);
    bool ok = true;
    int prev_first = 0;
    for (int a = a_lo - 1; a <= a_hi && ok; a++) {
        int count = 0;
        int first = 0;
        int last = 0;
        for (int b = b_lo - 1; b <= b_hi; b++) {
            bool in = x_major ? shape_contains(s, a, b) : shape_contains(s, b, a);
            if (in) {
                if (count == 0) {
                    first = b;
                }
                last = b;
                count++;
            }
        }
        bool in_range = a >= a1 && a <= a2;
        if (t <= 2 && a1 != a2) {
            ok = count == (in_range ? t : 0);
        } else if (in_range && a1 != a2) {
            ok = count >= t;
        }
        ok = ok && (count == 0 || last - first + 1 == count);
        if (ok && t == 1 && in_range && a > a1) {
            ok = abs(first - prev_first) <= 1;
        }
        prev_first = first;
    }
    ok = ok && shape_contains(s, x1, y1) && shape_contains(s, x2, y2);
    ok = ok && line_matches_reference(s, x1, y1, x2, y2, t);
    if (a1 == a2) {
        struct ShapeData *disc;
        int dr = (t - 1) / 2;
        if (dr > 0) {
            CHECK((disc = shape_new_ellipse(x1, y1, dr, dr)) != NULL);
            ok = ok && same_pixels(s, disc);
            shape_destroy(disc);
        } else {
            ok = ok && bw == 1 && bh == 1;
        }
    }
    if (!ok) {
        fprintf(stderr, "%s:%d: line (%d,%d)-(%d,%d) T=%d violates line properties\n", __FILE__,
            __LINE__, x1, y1, x2, y2, t);
        failures++;
    }
    CHECK_RUNS(s);

    struct ShapeData *r;
    CHECK((r = shape_new_line(x2, y2, x1, y1, t)) != NULL);
    CHECK(same_pixels(s, r));
    shape_destroy(r);
    shape_destroy(s);
}

static void test_line_runs_random(void)
{
    for (int i = 0; i < 2000; i++) {
        int major = rng_range(0, 80);
        int minor = rng_range(-major, major);
        int dx = rng_range(0, 1) ? major : -major;
        int dy = minor;
        if (rng_range(0, 1)) {
            int tmp = dx;
            dx = dy;
            dy = tmp;
        }
        int x1 = rng_range(-30, 30);
        int y1 = rng_range(-30, 30);
        check_line_properties(x1, y1, x1 + dx, y1 + dy, rng_range(1, 9));
    }
    for (int t = 1; t <= 4; t++) {
        check_line_properties(0, 0, 17, 17, t);
        check_line_properties(0, 0, -17, 17, t);
        check_line_properties(0, 0, 17, 16, t);
        check_line_properties(0, 0, 16, 17, t);
        check_line_properties(0, 0, 0, -9, t);
        check_line_properties(0, 0, 1, 0, t);
        check_line_properties(0, 0, 0, 0, t);
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

                    static const int angles[][2] = { { 0, 90 }, { 45, -45 }, { 0, 360 },
                        { 179, 181 }, { INT_MIN, INT_MAX } };
                    int k = (a + b) % 5;
                    CHECK((s = shape_new_arc(cx, cy, sizes[a], sizes[b], angles[k][0], angles[k][1])) != NULL);
                    check_extreme(s, "arc", __LINE__);
                    shape_destroy(s);
                }
            }
        }
    }

    const int ends[] = { -lim, 0, lim };
    const int thick[] = { 1, 2, 3, 4, lim - 1, lim };
    for (int p = 0; p < 9; p++) {
        for (int q = 0; q < 9; q++) {
            for (size_t t = 0; t < sizeof(thick) / sizeof(thick[0]); t++) {
                CHECK((s = shape_new_line(ends[p % 3], ends[p / 3], ends[q % 3], ends[q / 3], thick[t])) != NULL);
                check_extreme(s, "line", __LINE__);
                shape_destroy(s);
            }
        }
    }
    CHECK((s = shape_new_line(-lim, -lim + 1, lim, lim, 1)) != NULL);
    check_extreme(s, "line", __LINE__);
    shape_destroy(s);
    CHECK((s = shape_new_line(-lim, lim, lim - 3, -lim, 1)) != NULL);
    check_extreme(s, "line", __LINE__);
    shape_destroy(s);

    struct ShapePoint square[] = { { -lim, -lim }, { lim, -lim }, { lim, lim }, { -lim, lim } };
    struct ShapePoint bowtie[] = { { -lim, -lim }, { lim, lim }, { lim, -lim }, { -lim, lim } };
    struct ShapePoint sliver[] = { { -lim, -lim }, { lim, lim - 1 }, { lim, lim } };
    CHECK((s = shape_new_polygon(square, 4)) != NULL);
    check_extreme(s, "polygon", __LINE__);
    shape_destroy(s);
    CHECK((s = shape_new_polygon(bowtie, 4)) != NULL);
    check_extreme(s, "polygon", __LINE__);
    shape_destroy(s);
    CHECK((s = shape_new_polygon(sliver, 3)) != NULL);
    check_extreme(s, "polygon", __LINE__);
    shape_destroy(s);

    int n = SHAPE_POLYGON_MAX_POINTS;
    struct ShapePoint *comb = malloc(sizeof(struct ShapePoint) * n);
    for (int i = 0; i < n; i++) {
        comb[i].x = -lim + (int) ((int64_t) (i / 2) * 2 * lim / (n / 2));
        comb[i].y = ((i + 1) % 4 < 2) ? -lim : lim;
    }
    CHECK((s = shape_new_polygon(comb, n)) != NULL);
    check_extreme(s, "polygon", __LINE__);
    shape_destroy(s);
    free(comb);
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

    CHECK(shape_new_line(over, 0, 0, 0, 1) == NULL);
    CHECK(shape_new_line(0, under, 0, 0, 1) == NULL);
    CHECK(shape_new_line(0, 0, INT_MIN, 0, 1) == NULL);
    CHECK(shape_new_line(0, 0, 0, over, 1) == NULL);
    CHECK(shape_new_line(0, 0, 0, 0, over) == NULL);

    CHECK(shape_new_arc(over, 0, 1, 1, 0, 90) == NULL);
    CHECK(shape_new_arc(0, under, 1, 1, 0, 90) == NULL);
    CHECK(shape_new_arc(0, 0, over, 1, 0, 90) == NULL);
    CHECK(shape_new_arc(0, 0, 1, over, 0, 90) == NULL);

    struct ShapePoint pts[] = { { 0, 0 }, { over, 0 }, { 0, 4 } };
    CHECK(shape_new_polygon(pts, 3) == NULL);
    pts[1].x = 4;
    pts[2].y = under;
    CHECK(shape_new_polygon(pts, 3) == NULL);
    pts[2].y = SHAPE_VALUE_LIMIT;
    CHECK((s = shape_new_polygon(pts, 3)) != NULL);
    shape_destroy(s);

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
    test_line_horizontal_thickness();
    test_line_even_caps();
    test_line_diagonal_connected();
    test_line_shallow_one_pixel_per_column();
    test_line_thick_sloped();
    test_line_reversed_is_same();
    test_line_zero_length_is_dot();
    test_line_invalid();
    test_negative_coordinates();
    test_equal();
    test_arc_quarter();
    test_arc_full_ring();
    test_arc_over_180();
    test_arc_wraparound();
    test_arc_negative_angles();
    test_arc_sweep_rule();
    test_arc_pie();
    test_arc_matches_circle_when_full_pie();
    test_arc_ring_matches_circles();
    test_arc_tiny_sweep_visible();
    test_arc_invalid();
    test_large_values();
    test_polygon_square_matches_rect();
    test_polygon_concave();
    test_polygon_even_odd_hole();
    test_polygon_triangle_rows_out_of_order();
    test_polygon_copies_points();
    test_polygon_invalid();
    test_polygon_max_points();
    test_polygon_comb();
    test_polygon_equal();
    test_runs_match_contains();
    test_run_outside_bbox();
    test_arc_runs_random();
    test_polygon_runs_random();
    test_ellipse_runs_random();
    test_rounded_rect_runs_random();
    test_line_runs_random();
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
