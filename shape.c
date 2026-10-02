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

struct ShapeEllipse
{
    struct ShapeConvex convex;
    int cx;
    int cy;
    int rx;
    int ry;
};

struct ShapeLine
{
    struct ShapeConvex convex;
    int x1;
    int y1;
    int x2;
    int y2;
    int thickness;
    int cap_radius;
    bool x_major;
    int a1;
    int b1;
    int a2;
    int b2;
    int da;
    int db;
    int64_t t_da;
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

static int32_t floor_div32(int32_t n, int32_t d)
{
    int32_t q = n / d;
    if ((n % d) != 0 && n < 0) {
        q--;
    }
    return q;
}

static int64_t floor_div(int64_t n, int64_t d)
{
    if (n >= INT32_MIN && n <= INT32_MAX && d <= INT32_MAX) {
        return floor_div32((int32_t) n, (int32_t) d);
    }
    int64_t q = n / d;
    if ((n % d) != 0 && n < 0) {
        q--;
    }
    return q;
}

static int64_t ceil_div(int64_t n, int64_t d)
{
    return -floor_div(-n, d);
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

static void line_bounds(struct ShapeLine *l)
{
    int t = l->thickness;
    int r = (t - 1) / 2;
    int64_t dx = (int64_t) l->x2 - l->x1;
    int64_t dy = (int64_t) l->y2 - l->y1;
    int lo_x = int_min(l->x1, l->x2) - r;
    int hi_x = int_max(l->x1, l->x2) + r;
    int lo_y = int_min(l->y1, l->y2) - r;
    int hi_y = int_max(l->y1, l->y2) + r;
    if (dx != 0 || dy != 0) {
        if ((dx < 0 ? -dx : dx) >= (dy < 0 ? -dy : dy)) {
            lo_y = int_min(l->y1, l->y2) - t / 2;
        } else {
            lo_x = int_min(l->x1, l->x2) - t / 2;
        }
    }
    struct ShapeData *base = &l->convex.base;
    base->x = lo_x;
    base->y = lo_y;
    base->w = hi_x - lo_x + 1;
    base->h = hi_y - lo_y + 1;
}

static void line_frame(struct ShapeLine *l)
{
    int64_t dx = (int64_t) l->x2 - l->x1;
    int64_t dy = (int64_t) l->y2 - l->y1;
    l->x_major = (dx < 0 ? -dx : dx) >= (dy < 0 ? -dy : dy);
    int a1 = l->x_major ? l->x1 : l->y1;
    int b1 = l->x_major ? l->y1 : l->x1;
    int a2 = l->x_major ? l->x2 : l->y2;
    int b2 = l->x_major ? l->y2 : l->x2;
    if (a1 > a2) {
        l->a1 = a2;
        l->b1 = b2;
        l->a2 = a1;
        l->b2 = b1;
    } else {
        l->a1 = a1;
        l->b1 = b1;
        l->a2 = a2;
        l->b2 = b2;
    }
    l->da = l->a2 - l->a1;
    l->db = l->b2 - l->b1;
    l->t_da = (int64_t) l->thickness * l->da;
}

struct ShapeData *shape_new_line(int x1, int y1, int x2, int y2, int thickness)
{
    if (!in_limit(x1) || !in_limit(y1) || !in_limit(x2) || !in_limit(y2) || !in_limit(thickness)
        || thickness <= 0) {
        return NULL;
    }
    struct ShapeLine *l = shape_alloc(sizeof(*l), ShapeKindLine);
    if (l == NULL) {
        return NULL;
    }
    convex_init(&l->convex);
    l->x1 = x1;
    l->y1 = y1;
    l->x2 = x2;
    l->y2 = y2;
    l->thickness = thickness;
    l->cap_radius = (thickness - 1) / 2;
    line_bounds(l);
    line_frame(l);
    return &l->convex.base;
}

struct ShapeData *shape_new_ellipse(int cx, int cy, int rx, int ry)
{
    if (!in_limit(cx) || !in_limit(cy) || !in_limit(rx) || !in_limit(ry) || rx <= 0 || ry <= 0) {
        return NULL;
    }
    struct ShapeEllipse *e = shape_alloc(sizeof(*e), ShapeKindEllipse);
    if (e == NULL) {
        return NULL;
    }
    convex_init(&e->convex);
    e->cx = cx;
    e->cy = cy;
    e->rx = rx;
    e->ry = ry;
    e->convex.base.x = cx - rx;
    e->convex.base.y = cy - ry;
    e->convex.base.w = 2 * rx + 1;
    e->convex.base.h = 2 * ry + 1;
    return &e->convex.base;
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

static bool ellipse_contains(const struct ShapeEllipse *e, int x, int y)
{
    if (!in_bbox(&e->convex.base, x, y)) {
        return false;
    }
    int64_t dx = (int64_t) x - e->cx;
    int64_t dy = (int64_t) y - e->cy;
    int64_t rx = e->rx;
    int64_t ry = e->ry;
    // midpoint criterion dx^2 / (rx^2 + rx) + dy^2 / (ry^2 + ry) < 1
    int64_t kx = rx * rx + rx;
    int64_t ky = ry * ry + ry;
    return dx * dx * ky + dy * dy * kx < kx * ky;
}

static bool line_in_cap(const struct ShapeLine *l, int64_t da, int64_t db)
{
    int64_t t = l->thickness;
    int64_t c = (t % 2 == 0) ? 1 : 0;
    int64_t u = 2 * da;
    int64_t v = 2 * db + c;
    return u * u + v * v < t * t - 1;
}

static bool line_contains(const struct ShapeLine *l, int x, int y)
{
    if (!in_bbox(&l->convex.base, x, y)) {
        return false;
    }
    if (l->da == 0) {
        int64_t dx = (int64_t) x - l->x1;
        int64_t dy = (int64_t) y - l->y1;
        return (dx == 0 && dy == 0) || in_disc(dx, dy, l->cap_radius);
    }
    int a = l->x_major ? x : y;
    int b = l->x_major ? y : x;
    if (line_in_cap(l, (int64_t) a - l->a1, (int64_t) b - l->b1)
        || line_in_cap(l, (int64_t) a - l->a2, (int64_t) b - l->b2)) {
        return true;
    }
    if (a < l->a1 || a > l->a2) {
        return false;
    }
    int64_t e = 2 * (((int64_t) b - l->b1) * l->da - ((int64_t) a - l->a1) * l->db);
    return -l->t_da <= e && e < l->t_da;
}

bool shape_contains(struct ShapeData *shape, int x, int y)
{
    switch (shape->kind) {
        case ShapeKindRoundedRect:
            return rounded_rect_contains((const struct ShapeRoundedRect *) shape, x, y);
        case ShapeKindLine:
            return line_contains((const struct ShapeLine *) shape, x, y);
        case ShapeKindEllipse:
            return ellipse_contains((const struct ShapeEllipse *) shape, x, y);
        default:
            return false;
    }
}

static int line_candidate_x(const struct ShapeLine *l, int y, int *last)
{
    int64_t da = l->da;
    int64_t db = l->db;
    int64_t t = l->t_da;
    int r = l->cap_radius;

    if (da > 0) {
        if (l->x_major) {
            int64_t k = 2 * ((int64_t) y - l->b1) * da;
            int64_t lo;
            int64_t hi;
            if (db > 0) {
                lo = floor_div(k - t, 2 * db) + 1;
                hi = floor_div(k + t, 2 * db);
            } else if (db < 0) {
                lo = ceil_div(-(k + t), -2 * db);
                hi = ceil_div(t - k, -2 * db) - 1;
            } else if (-t <= k && k < t) {
                lo = 0;
                hi = da;
            } else {
                lo = 1;
                hi = 0;
            }
            lo = lo < 0 ? 0 : lo;
            hi = hi > da ? da : hi;
            if (lo <= hi) {
                *last = (int) (l->a1 + hi);
                return (int) (l->a1 + lo);
            }
        } else if (y >= l->a1 && y <= l->a2) {
            int64_t j = 2 * ((int64_t) y - l->a1) * db;
            *last = (int) (l->b1 + ceil_div(j + t, 2 * da) - 1);
            return (int) (l->b1 + ceil_div(j - t, 2 * da));
        }
    }
    int64_t dy1 = (int64_t) y - l->y1;
    *last = (dy1 >= -r && dy1 <= r) ? l->x1 : l->x2;
    return *last;
}

static int convex_candidate_x(const struct ShapeData *shape)
{
    switch (shape->kind) {
        case ShapeKindRoundedRect:
            return shape->x + shape->w / 2;
        case ShapeKindEllipse:
            return ((const struct ShapeEllipse *) shape)->cx;
        default:
            return 0;
    }
}

static int line_row_edge(struct ShapeData *shape, int y, int x, int limit, int dir)
{
    int step = 1;
    while ((limit - x) * dir >= step && shape_contains(shape, x + step * dir, y)) {
        x += step * dir;
        step *= 2;
    }
    int far = ((limit - x) * dir >= step) ? step - 1 : (limit - x) * dir;
    int lo = 0;
    while (lo < far) {
        int mid = lo + (far - lo + 1) / 2;
        if (shape_contains(shape, x + mid * dir, y)) {
            lo = mid;
        } else {
            far = mid - 1;
        }
    }
    return x + lo * dir;
}

static void convex_update_row(struct ShapeConvex *convex, int y)
{
    struct ShapeData *shape = &convex->base;
    int bx = shape->x;
    int end = shape->x + shape->w;
    int line_last = 0;
    int candidate = (shape->kind == ShapeKindLine)
        ? line_candidate_x((const struct ShapeLine *) shape, y, &line_last)
        : convex_candidate_x(shape);
    candidate = int_max(bx, int_min(candidate, end - 1));

    int left;
    int right;
    if (!shape_contains(shape, candidate, y)) {
        left = bx;
        right = bx;
    } else if (shape->kind == ShapeKindLine) {
        int last = int_max(candidate, int_min(line_last, end - 1));
        if (!shape_contains(shape, last, y)) {
            last = candidate;
        }
        left = line_row_edge(shape, y, candidate, bx, -1);
        right = line_row_edge(shape, y, last, end - 1, 1) + 1;
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
        case ShapeKindLine:
        case ShapeKindEllipse:
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
        case ShapeKindLine: {
            const struct ShapeLine *l_a = (const struct ShapeLine *) a;
            const struct ShapeLine *l_b = (const struct ShapeLine *) b;
            return l_a->x1 == l_b->x1 && l_a->y1 == l_b->y1 && l_a->x2 == l_b->x2
                && l_a->y2 == l_b->y2 && l_a->thickness == l_b->thickness;
        }
        case ShapeKindEllipse: {
            const struct ShapeEllipse *e_a = (const struct ShapeEllipse *) a;
            const struct ShapeEllipse *e_b = (const struct ShapeEllipse *) b;
            return e_a->cx == e_b->cx && e_a->cy == e_b->cy && e_a->rx == e_b->rx
                && e_a->ry == e_b->ry;
        }
        default:
            return false;
    }
}

void shape_destroy(struct ShapeData *shape)
{
    free(shape);
}
