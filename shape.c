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

#define ARC_MAX_SEGMENTS 12

struct ShapeArc
{
    struct ShapeData base;
    int cx;
    int cy;
    int radius;
    int thickness;
    int sweep;
    int32_t start_vx;
    int32_t start_vy;
    int32_t end_vx;
    int32_t end_vy;
    int seg_row;
    int seg_count;
    int seg_start[ARC_MAX_SEGMENTS + 1];
    bool seg_inside[ARC_MAX_SEGMENTS];
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

// sin(d) for d = 0..90 degrees, Q14 (16384 = 1.0)
static const int16_t sin_q14[91] = {
    0, 286, 572, 857, 1143, 1428, 1713, 1997, 2280, 2563,
    2845, 3126, 3406, 3686, 3964, 4240, 4516, 4790, 5063, 5334,
    5604, 5872, 6138, 6402, 6664, 6924, 7182, 7438, 7692, 7943,
    8192, 8438, 8682, 8923, 9162, 9397, 9630, 9860, 10087, 10311,
    10531, 10749, 10963, 11174, 11381, 11585, 11786, 11982, 12176, 12365,
    12551, 12733, 12911, 13085, 13255, 13421, 13583, 13741, 13894, 14044,
    14189, 14330, 14466, 14598, 14726, 14849, 14968, 15082, 15191, 15296,
    15396, 15491, 15582, 15668, 15749, 15826, 15897, 15964, 16026, 16083,
    16135, 16182, 16225, 16262, 16294, 16322, 16344, 16362, 16374, 16382,
    16384
};

static int normalize_deg(int deg)
{
    deg %= 360;
    return (deg < 0) ? deg + 360 : deg;
}

static int32_t sin_deg(int deg)
{
    deg = normalize_deg(deg);
    if (deg <= 90) {
        return sin_q14[deg];
    } else if (deg <= 180) {
        return sin_q14[180 - deg];
    } else if (deg <= 270) {
        return -sin_q14[deg - 180];
    } else {
        return -sin_q14[360 - deg];
    }
}

static int32_t cos_deg(int deg)
{
    return sin_deg(normalize_deg(deg) + 90);
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

static int32_t ceil_div32(int32_t n, int32_t d)
{
    return -floor_div32(-n, d);
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

struct ShapeData *shape_new_arc(int cx, int cy, int radius, int thickness, int start_deg,
    int end_deg)
{
    if (!in_limit(cx) || !in_limit(cy) || !in_limit(radius) || !in_limit(thickness) || radius <= 0
        || thickness <= 0 || start_deg == end_deg) {
        return NULL;
    }
    int sweep = normalize_deg(normalize_deg(end_deg) - normalize_deg(start_deg));
    if (sweep == 0) {
        sweep = 360;
    }

    struct ShapeArc *arc = shape_alloc(sizeof(*arc), ShapeKindArc);
    if (arc == NULL) {
        return NULL;
    }
    arc->base.x = cx - radius;
    arc->base.y = cy - radius;
    arc->base.w = 2 * radius + 1;
    arc->base.h = 2 * radius + 1;
    arc->cx = cx;
    arc->cy = cy;
    arc->radius = radius;
    arc->thickness = thickness;
    arc->sweep = sweep;
    arc->start_vx = cos_deg(start_deg);
    arc->start_vy = sin_deg(start_deg);
    arc->end_vx = cos_deg(end_deg);
    arc->end_vy = sin_deg(end_deg);
    arc->seg_row = INT_MIN;
    return &arc->base;
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

static inline int64_t cross(int64_t ux, int64_t uy, int64_t vx, int64_t vy)
{
    return ux * vy - uy * vx;
}

static bool arc_in_outer(const struct ShapeArc *arc, int x, int y)
{
    return in_disc((int64_t) x - arc->cx, (int64_t) y - arc->cy, arc->radius);
}

static bool arc_in_hole(const struct ShapeArc *arc, int x, int y)
{
    int inner_radius = arc->radius - arc->thickness;
    if (inner_radius <= 0) {
        return false;
    }
    return in_disc((int64_t) x - arc->cx, (int64_t) y - arc->cy, inner_radius);
}

static bool arc_cross_a(const struct ShapeArc *arc, int x, int y)
{
    int64_t dx = (int64_t) x - arc->cx;
    int64_t dy = (int64_t) y - arc->cy;
    if (arc->sweep <= 180) {
        return cross(arc->start_vx, arc->start_vy, dx, dy) >= 0;
    }
    return cross(arc->end_vx, arc->end_vy, dx, dy) > 0;
}

static bool arc_cross_b(const struct ShapeArc *arc, int x, int y)
{
    int64_t dx = (int64_t) x - arc->cx;
    int64_t dy = (int64_t) y - arc->cy;
    if (arc->sweep <= 180) {
        return cross(dx, dy, arc->end_vx, arc->end_vy) >= 0;
    }
    return cross(dx, dy, arc->start_vx, arc->start_vy) > 0;
}

static bool arc_on_ray(int32_t vx, int32_t vy, int64_t dx, int64_t dy)
{
    int64_t c = cross(vx, vy, dx, dy);
    int64_t s = (int64_t) (vx < 0 ? -vx : vx) + (vy < 0 ? -vy : vy);
    return 2 * (c < 0 ? -c : c) <= s && vx * dx + vy * dy >= 0;
}

static bool arc_contains(const struct ShapeArc *arc, int x, int y)
{
    if (!in_bbox(&arc->base, x, y)) {
        return false;
    }
    if (!arc_in_outer(arc, x, y) || arc_in_hole(arc, x, y)) {
        return false;
    }
    if (arc->sweep == 360) {
        return true;
    }
    bool in_sweep;
    if (arc->sweep <= 180) {
        in_sweep = arc_cross_a(arc, x, y) && arc_cross_b(arc, x, y);
    } else {
        in_sweep = !(arc_cross_a(arc, x, y) && arc_cross_b(arc, x, y));
    }
    int64_t dx = (int64_t) x - arc->cx;
    int64_t dy = (int64_t) y - arc->cy;
    return in_sweep || arc_on_ray(arc->start_vx, arc->start_vy, dx, dy)
        || arc_on_ray(arc->end_vx, arc->end_vy, dx, dy);
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
        case ShapeKindArc:
            return arc_contains((const struct ShapeArc *) shape, x, y);
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

static int find_change(const struct ShapeArc *arc, int y, int lo, int hi,
    bool (*pred)(const struct ShapeArc *, int, int))
{
    if (lo >= hi) {
        return hi;
    }
    bool base = pred(arc, lo, y);
    int l = lo;
    int h = hi;
    while (l < h) {
        int mid = l + (h - l) / 2;
        if (pred(arc, mid, y) == base) {
            l = mid + 1;
        } else {
            h = mid;
        }
    }
    return l;
}

static void arc_add_breakpoint(int *bps, int *n, int x, int bx, int end)
{
    if (x < bx) {
        x = bx;
    } else if (x > end) {
        x = end;
    }
    for (int i = 0; i < *n; i++) {
        if (bps[i] == x) {
            return;
        }
    }
    int i = *n;
    while (i > 0 && bps[i - 1] > x) {
        bps[i] = bps[i - 1];
        i--;
    }
    bps[i] = x;
    (*n)++;
}

static void arc_add_ray_breakpoints(const struct ShapeArc *arc, int32_t vx, int32_t vy, int y,
    int *bps, int *n)
{
    int32_t dy = y - arc->cy;
    int32_t k = 2 * vx * dy;
    int32_t s = (vx < 0 ? -vx : vx) + (vy < 0 ? -vy : vy);
    int32_t lo = -arc->radius;
    int32_t hi = arc->radius;
    if (vy > 0) {
        lo = int_max(lo, ceil_div32(k - s, 2 * vy));
        hi = int_min(hi, floor_div32(k + s, 2 * vy));
    } else if (vy < 0) {
        lo = int_max(lo, ceil_div32(-(k + s), -2 * vy));
        hi = int_min(hi, floor_div32(s - k, -2 * vy));
    } else if (k < -s || k > s) {
        return;
    }
    int32_t m = vy * dy;
    if (vx > 0) {
        lo = int_max(lo, ceil_div32(-m, vx));
    } else if (vx < 0) {
        hi = int_min(hi, floor_div32(m, -vx));
    } else if (m < 0) {
        return;
    }
    if (lo <= hi) {
        int bx = arc->base.x;
        int end = arc->base.x + arc->base.w;
        arc_add_breakpoint(bps, n, arc->cx + lo, bx, end);
        arc_add_breakpoint(bps, n, arc->cx + hi + 1, bx, end);
    }
}

static void arc_update_row(struct ShapeArc *arc, int y)
{
    int bx = arc->base.x;
    int end = arc->base.x + arc->base.w;
    _Static_assert(13 <= ARC_MAX_SEGMENTS + 1, "bps too small for arc_update_row's breakpoint inserts");
    int bps[ARC_MAX_SEGMENTS + 1];
    int n = 0;
    arc_add_breakpoint(bps, &n, bx, bx, end);
    arc_add_breakpoint(bps, &n, end, bx, end);

    int c = arc->cx;
    if (c < bx) {
        c = bx;
    } else if (c > end) {
        c = end;
    }
    arc_add_breakpoint(bps, &n, c, bx, end);

    bool have_hole = arc->radius - arc->thickness > 0;
    if (bx < c) {
        arc_add_breakpoint(bps, &n, find_change(arc, y, bx, c, arc_in_outer), bx, end);
        if (have_hole) {
            arc_add_breakpoint(bps, &n, find_change(arc, y, bx, c, arc_in_hole), bx, end);
        }
    }
    if (c < end) {
        arc_add_breakpoint(bps, &n, find_change(arc, y, c, end, arc_in_outer), bx, end);
        if (have_hole) {
            arc_add_breakpoint(bps, &n, find_change(arc, y, c, end, arc_in_hole), bx, end);
        }
    }

    if (arc->sweep != 360) {
        arc_add_breakpoint(bps, &n, find_change(arc, y, bx, end, arc_cross_a), bx, end);
        arc_add_breakpoint(bps, &n, find_change(arc, y, bx, end, arc_cross_b), bx, end);
        arc_add_ray_breakpoints(arc, arc->start_vx, arc->start_vy, y, bps, &n);
        arc_add_ray_breakpoints(arc, arc->end_vx, arc->end_vy, y, bps, &n);
    }

    int seg_count = 0;
    arc->seg_start[0] = bps[0];
    bool prev_inside = arc_contains(arc, bps[0], y);
    for (int i = 1; i < n; i++) {
        bool inside = arc_contains(arc, bps[i], y);
        if (inside != prev_inside) {
            arc->seg_inside[seg_count] = prev_inside;
            seg_count++;
            arc->seg_start[seg_count] = bps[i];
            prev_inside = inside;
        }
    }
    arc->seg_inside[seg_count] = prev_inside;
    seg_count++;
    arc->seg_start[seg_count] = end;

    arc->seg_count = seg_count;
    arc->seg_row = y;
}

static int arc_run(struct ShapeArc *arc, int x, int y, bool *inside)
{
    if (arc->seg_row != y) {
        arc_update_row(arc, y);
    }

    int i = 0;
    while (i + 1 < arc->seg_count && arc->seg_start[i + 1] <= x) {
        i++;
    }
    *inside = arc->seg_inside[i];
    return arc->seg_start[i + 1] - x;
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
        case ShapeKindArc:
            return arc_run((struct ShapeArc *) shape, x, y, inside);
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
        case ShapeKindArc: {
            const struct ShapeArc *arc_a = (const struct ShapeArc *) a;
            const struct ShapeArc *arc_b = (const struct ShapeArc *) b;
            return arc_a->cx == arc_b->cx && arc_a->cy == arc_b->cy
                && arc_a->radius == arc_b->radius && arc_a->thickness == arc_b->thickness
                && arc_a->sweep == arc_b->sweep && arc_a->start_vx == arc_b->start_vx
                && arc_a->start_vy == arc_b->start_vy && arc_a->end_vx == arc_b->end_vx
                && arc_a->end_vy == arc_b->end_vy;
        }
        default:
            return false;
    }
}

void shape_destroy(struct ShapeData *shape)
{
    free(shape);
}
