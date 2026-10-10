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

#ifndef _SHAPE_H_
#define _SHAPE_H_

#include <stdbool.h>
#include <stdint.h>

#define SHAPE_VALUE_LIMIT 32767

#define SHAPE_POLYGON_MAX_POINTS 64

typedef enum
{
    ShapeKindRoundedRect,
    ShapeKindLine,
    ShapeKindEllipse,
    ShapeKindArc,
    ShapeKindPolygon
} shape_kind_t;

struct ShapePoint
{
    int x;
    int y;
};

struct ShapeData;

struct ShapeData *shape_new_rounded_rect(int x, int y, int w, int h, int radius);
struct ShapeData *shape_new_line(int x1, int y1, int x2, int y2, int thickness);
struct ShapeData *shape_new_ellipse(int cx, int cy, int rx, int ry);
struct ShapeData *shape_new_arc(int cx, int cy, int radius, int thickness, int start_deg,
    int end_deg);
struct ShapeData *shape_new_polygon(const struct ShapePoint *points, int points_len);

// Builds a polygon in place, without an array of points: shape_polygon_begin() allocates it for
// points_len vertices, shape_polygon_add_point() appends each of them in order and
// shape_polygon_end() closes the outline once all of them are in. Destroy the shape if either
// returns false.
struct ShapeData *shape_polygon_begin(int points_len);
bool shape_polygon_add_point(struct ShapeData *shape, int x, int y);
bool shape_polygon_end(struct ShapeData *shape);
void shape_destroy(struct ShapeData *shape);

shape_kind_t shape_kind(const struct ShapeData *shape);
bool shape_contains(const struct ShapeData *shape, int x, int y);
// Run of pixels from (x, y) with the same inside/outside state: at least 1, never past the
// bounding box; *inside tells which. Updates the shape's row cache, so rows in order are the
// fast path (polygons rebuild on a jump) and one shape must not be walked by two renderers at
// once.
int shape_run(struct ShapeData *shape, int x, int y, bool *inside);
void shape_bounds(const struct ShapeData *shape, int *x, int *y, int *w, int *h);
bool shape_equal(const struct ShapeData *a, const struct ShapeData *b);

#endif
