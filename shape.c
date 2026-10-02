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

#include "shape.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(SHAPE_VALUE_LIMIT == 32767, "shape.c's overflow bounds assume SHAPE_VALUE_LIMIT is 32767");

struct ShapeData
{
    shape_kind_t kind;
    int x;
    int y;
    int w;
    int h;
};

struct ShapeRowCache
{
    int row;
    int left;
    int right;
};

struct ShapeConvex
{
    struct ShapeData base;
    struct ShapeRowCache row_cache;
};

struct ShapeRoundedRect
{
    struct ShapeConvex convex;
    int radius;
};

static void *shape_alloc(size_t size, shape_kind_t kind)
{
    struct ShapeData *shape = malloc(size);
    if (shape != NULL) {
        memset(shape, 0, size);
        shape->kind = kind;
    }
    return shape;
}

static inline int int_min(int a, int b)
{
    return (a > b) ? b : a;
}

static inline int int_max(int a, int b)
{
    return (a > b) ? a : b;
}

static inline bool in_limit(int v)
{
    return v >= -SHAPE_VALUE_LIMIT && v <= SHAPE_VALUE_LIMIT;
}

static void convex_init(struct ShapeConvex *convex)
{
    convex->row_cache.row = INT_MIN;
}

struct ShapeData *shape_new_rounded_rect(int x, int y, int w, int h, int radius)
{
    if (!in_limit(x) || !in_limit(y) || !in_limit(w) || !in_limit(h) || !in_limit(radius)
        || w <= 0 || h <= 0 || radius < 0) {
        return NULL;
    }
    struct ShapeRoundedRect *rr = shape_alloc(sizeof(*rr), ShapeKindRoundedRect);
    if (rr == NULL) {
        return NULL;
    }
    convex_init(&rr->convex);
    rr->convex.base.x = x;
    rr->convex.base.y = y;
    rr->convex.base.w = w;
    rr->convex.base.h = h;
    rr->radius = int_min(radius, (int_min(w, h) - 1) / 2);
    return &rr->convex.base;
}

static inline bool in_disc(int64_t dx, int64_t dy, int64_t r)
{
    return dx * dx + dy * dy < r * r + r;
}

static inline bool in_bbox(const struct ShapeData *b, int x, int y)
{
    return x >= b->x && x < b->x + b->w && y >= b->y && y < b->y + b->h;
}

static bool rounded_rect_contains(const struct ShapeRoundedRect *rr, int x, int y)
{
    const struct ShapeData *b = &rr->convex.base;
    if (!in_bbox(b, x, y)) {
        return false;
    }
    int r = rr->radius;
    if (r == 0) {
        return true;
    }

    int left = b->x + r;
    int right = b->x + b->w - 1 - r;
    int top = b->y + r;
    int bottom = b->y + b->h - 1 - r;

    int cx;
    int cy;
    if (x < left) {
        cx = left;
    } else if (x > right) {
        cx = right;
    } else {
        return true;
    }
    if (y < top) {
        cy = top;
    } else if (y > bottom) {
        cy = bottom;
    } else {
        return true;
    }
    return in_disc(x - cx, y - cy, r);
}

bool shape_contains(struct ShapeData *shape, int x, int y)
{
    switch (shape->kind) {
        case ShapeKindRoundedRect:
            return rounded_rect_contains((const struct ShapeRoundedRect *) shape, x, y);
        default:
            return false;
    }
}

static int convex_candidate_x(const struct ShapeData *shape)
{
    switch (shape->kind) {
        case ShapeKindRoundedRect:
            return shape->x + shape->w / 2;
        default:
            return 0;
    }
}

static void convex_update_row(struct ShapeConvex *convex, int y)
{
    struct ShapeData *shape = &convex->base;
    int bx = shape->x;
    int end = shape->x + shape->w;
    int candidate = convex_candidate_x(shape);
    candidate = int_max(bx, int_min(candidate, end - 1));

    int left;
    int right;
    if (!shape_contains(shape, candidate, y)) {
        left = bx;
        right = bx;
    } else {
        int lo = bx;
        int hi = candidate;
        while (lo < hi) {
            int mid = lo + (hi - lo) / 2;
            if (shape_contains(shape, mid, y)) {
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        left = lo;

        lo = candidate + 1;
        hi = end;
        while (lo < hi) {
            int mid = lo + (hi - lo) / 2;
            if (shape_contains(shape, mid, y)) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        right = lo;
    }

    convex->row_cache.row = y;
    convex->row_cache.left = left;
    convex->row_cache.right = right;
}

static int convex_run(struct ShapeConvex *convex, int x, int y, int end, bool *inside)
{
    if (convex->row_cache.row != y) {
        convex_update_row(convex, y);
    }
    int left = convex->row_cache.left;
    int right = convex->row_cache.right;

    if (x < left) {
        *inside = false;
        return left - x;
    }
    if (x < right) {
        *inside = true;
        return right - x;
    }
    *inside = false;
    return end - x;
}

int shape_run(struct ShapeData *shape, int x, int y, bool *inside)
{
    int bx = shape->x;
    int end = shape->x + shape->w;
    if (y < shape->y || y >= shape->y + shape->h || x >= end) {
        *inside = false;
        return 1;
    }
    if (x < bx) {
        *inside = false;
        int64_t gap = (int64_t) bx - x;
        return (gap > INT_MAX) ? INT_MAX : (int) gap;
    }

    switch (shape->kind) {
        case ShapeKindRoundedRect:
            return convex_run((struct ShapeConvex *) shape, x, y, end, inside);
        default:
            *inside = false;
            return 1;
    }
}

shape_kind_t shape_kind(const struct ShapeData *shape)
{
    return shape->kind;
}

void shape_bounds(const struct ShapeData *shape, int *x, int *y, int *w, int *h)
{
    *x = shape->x;
    *y = shape->y;
    *w = shape->w;
    *h = shape->h;
}

bool shape_equal(const struct ShapeData *a, const struct ShapeData *b)
{
    if (a->kind != b->kind) {
        return false;
    }
    switch (a->kind) {
        case ShapeKindRoundedRect: {
            const struct ShapeRoundedRect *rr_a = (const struct ShapeRoundedRect *) a;
            const struct ShapeRoundedRect *rr_b = (const struct ShapeRoundedRect *) b;
            return a->x == b->x && a->y == b->y && a->w == b->w && a->h == b->h
                && rr_a->radius == rr_b->radius;
        }
        default:
            return false;
    }
}

void shape_destroy(struct ShapeData *shape)
{
    free(shape);
}
