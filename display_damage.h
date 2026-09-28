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

#ifndef _DISPLAY_DAMAGE_H_
#define _DISPLAY_DAMAGE_H_

#include <stdbool.h>
#include <stddef.h>

#define DISPLAY_DAMAGE_MAX_RECTS 8

struct DisplayRect
{
    int x;
    int y;
    int width;
    int height;
};

// Set of screen areas that must be repainted.
//
// Rectangles are always clipped to the screen and never empty; they may
// overlap each other (repainting a pixel twice is harmless). Once
// DISPLAY_DAMAGE_MAX_RECTS rectangles are stored, further ones are merged
// into the existing rectangle that grows the least, so the set is always a
// superset of everything that was added.
struct DisplayDamage
{
    int screen_width;
    int screen_height;
    size_t rects_len;
    struct DisplayRect rects[DISPLAY_DAMAGE_MAX_RECTS];
};

void display_damage_init(struct DisplayDamage *damage, int screen_width, int screen_height);
void display_damage_clear(struct DisplayDamage *damage);

// Adds a rectangle; parts outside the screen are dropped, empty rectangles
// are ignored.
void display_damage_add_rect(struct DisplayDamage *damage, int x, int y, int width, int height);
void display_damage_add_all(struct DisplayDamage *damage);
void display_damage_add_damage(struct DisplayDamage *damage, const struct DisplayDamage *other);

bool display_damage_is_empty(const struct DisplayDamage *damage);
bool display_damage_is_full(const struct DisplayDamage *damage);

// Reports whether any rectangle touches the rows [y_start, y_end). When it
// does, [*x_start, *x_end) is set to the horizontal span covering every such
// rectangle.
bool display_damage_rows_span(const struct DisplayDamage *damage, int y_start, int y_end,
    int *x_start, int *x_end);

#endif
