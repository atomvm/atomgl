/*
 * This file is part of AtomGL.
 *
 * Copyright 2022-2026 Davide Bettio <davide@uninstall.it>
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

#ifndef _EPAPER_DRAW_H_
#define _EPAPER_DRAW_H_

#include "epaper_screen.h"
#include "display_items.h"

void epaper_draw_pixel_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, uint8_t c);

int epaper_draw_image_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item);

int epaper_draw_rect_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item);

int epaper_draw_shape_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item, int *outside_run);

int epaper_draw_text_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item);

int epaper_draw_scaled_cropped_img_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos, int max_line_len,
    BaseDisplayItem *item);

int epaper_draw_x(const struct EpaperScreen *screen,
    uint8_t *line_buf, int xpos, int ypos,
    BaseDisplayItem *row);

#endif
