/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2026 Davide Bettio <davide@uninstall.it>
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

#ifndef _DCS_LCD_DRAW_H_
#define _DCS_LCD_DRAW_H_

#include "dcs_lcd_screen.h"
#include "display_items.h"

int dcs_lcd_draw_image_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item);

int dcs_lcd_draw_rect_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item);

int dcs_lcd_draw_shape_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item, int *outside_run);

int dcs_lcd_draw_text_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item);

int dcs_lcd_draw_scaled_cropped_img_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, int max_line_len, BaseDisplayItem *item);

int dcs_lcd_draw_x(const struct DCSLCDScreen *screen,
    int xpos, int ypos, BaseDisplayItem *row);

#endif
