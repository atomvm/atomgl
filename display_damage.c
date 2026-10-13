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

#include <stdbool.h>
#include <stddef.h>

static inline int int_min(int a, int b)
{
    return (a < b) ? a : b;
}

static inline int int_max(int a, int b)
{
    return (a > b) ? a : b;
}

static inline long long llong_min(long long a, long long b)
{
    return (a < b) ? a : b;
}

static inline long long llong_max(long long a, long long b)
{
    return (a > b) ? a : b;
}

static bool rect_contains(const struct DisplayRect *outer, const struct DisplayRect *inner)
{
    return inner->x >= outer->x && inner->y >= outer->y
        && inner->x + inner->width <= outer->x + outer->width
        && inner->y + inner->height <= outer->y + outer->height;
}

static struct DisplayRect rect_union(const struct DisplayRect *a, const struct DisplayRect *b)
{
    int x = int_min(a->x, b->x);
    int y = int_min(a->y, b->y);
    int x_end = int_max(a->x + a->width, b->x + b->width);
    int y_end = int_max(a->y + a->height, b->y + b->height);

    struct DisplayRect result = { .x = x, .y = y, .width = x_end - x, .height = y_end - y };

    return result;
}

static long rect_area(const struct DisplayRect *rect)
{
    return (long) rect->width * rect->height;
}

void display_damage_init(struct DisplayDamage *damage, int screen_width, int screen_height)
{
    damage->screen_width = screen_width;
    damage->screen_height = screen_height;
    damage->rects_len = 0;
}

void display_damage_clear(struct DisplayDamage *damage)
{
    damage->rects_len = 0;
}

static void remove_contained_rects(struct DisplayDamage *damage, const struct DisplayRect *rect)
{
    size_t kept = 0;
    for (size_t i = 0; i < damage->rects_len; i++) {
        if (!rect_contains(rect, &damage->rects[i])) {
            damage->rects[kept] = damage->rects[i];
            kept++;
        }
    }
    damage->rects_len = kept;
}

// `rect` must be non-empty and within the screen.
static void add_clipped_rect(struct DisplayDamage *damage, struct DisplayRect rect)
{
    for (size_t i = 0; i < damage->rects_len; i++) {
        if (rect_contains(&damage->rects[i], &rect)) {
            return;
        }
    }
    remove_contained_rects(damage, &rect);

    if (damage->rects_len < DISPLAY_DAMAGE_MAX_RECTS) {
        damage->rects[damage->rects_len] = rect;
        damage->rects_len++;
        return;
    }

    size_t best = 0;
    long best_waste = 0;
    for (size_t i = 0; i < damage->rects_len; i++) {
        struct DisplayRect merged = rect_union(&damage->rects[i], &rect);
        long waste = rect_area(&merged) - rect_area(&damage->rects[i]) - rect_area(&rect);
        if (i == 0 || waste < best_waste) {
            best = i;
            best_waste = waste;
        }
    }

    struct DisplayRect merged = rect_union(&damage->rects[best], &rect);
    // The merged rectangle contains rects[best], so this drops it as well.
    remove_contained_rects(damage, &merged);
    damage->rects[damage->rects_len] = merged;
    damage->rects_len++;
}

void display_damage_add_rect(struct DisplayDamage *damage, int x, int y, int width, int height)
{
    // Wide arithmetic: items may carry arbitrary coordinates and sizes.
    long long x_start = llong_max(x, 0);
    long long y_start = llong_max(y, 0);
    long long x_end = llong_min((long long) x + width, damage->screen_width);
    long long y_end = llong_min((long long) y + height, damage->screen_height);

    if (x_end <= x_start || y_end <= y_start) {
        return;
    }

    struct DisplayRect rect = {
        .x = (int) x_start,
        .y = (int) y_start,
        .width = (int) (x_end - x_start),
        .height = (int) (y_end - y_start)
    };
    add_clipped_rect(damage, rect);
}

void display_damage_add_all(struct DisplayDamage *damage)
{
    display_damage_add_rect(damage, 0, 0, damage->screen_width, damage->screen_height);
}

void display_damage_add_damage(struct DisplayDamage *damage, const struct DisplayDamage *other)
{
    for (size_t i = 0; i < other->rects_len; i++) {
        const struct DisplayRect *rect = &other->rects[i];
        display_damage_add_rect(damage, rect->x, rect->y, rect->width, rect->height);
    }
}

bool display_damage_is_empty(const struct DisplayDamage *damage)
{
    return damage->rects_len == 0;
}

bool display_damage_is_full(const struct DisplayDamage *damage)
{
    if (damage->rects_len != 1) {
        return false;
    }

    const struct DisplayRect *rect = &damage->rects[0];

    return rect->x == 0 && rect->y == 0 && rect->width == damage->screen_width
        && rect->height == damage->screen_height;
}

bool display_damage_rows_span(const struct DisplayDamage *damage, int y_start, int y_end,
    int *x_start, int *x_end)
{
    bool found = false;
    int span_start = 0;
    int span_end = 0;

    for (size_t i = 0; i < damage->rects_len; i++) {
        const struct DisplayRect *rect = &damage->rects[i];
        if (rect->y >= y_end || rect->y + rect->height <= y_start) {
            continue;
        }

        if (!found) {
            span_start = rect->x;
            span_end = rect->x + rect->width;
            found = true;
        } else {
            span_start = int_min(span_start, rect->x);
            span_end = int_max(span_end, rect->x + rect->width);
        }
    }

    if (found) {
        *x_start = span_start;
        *x_end = span_end;
    }

    return found;
}

bool display_damage_intersects_rect(const struct DisplayDamage *damage, int x, int y, int width,
    int height)
{
    if (width <= 0 || height <= 0) {
        return false;
    }
    long long x_end = (long long) x + width;
    long long y_end = (long long) y + height;

    for (size_t i = 0; i < damage->rects_len; i++) {
        const struct DisplayRect *rect = &damage->rects[i];
        if (x < rect->x + rect->width && x_end > rect->x && y < rect->y + rect->height
            && y_end > rect->y) {
            return true;
        }
    }

    return false;
}
