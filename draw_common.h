/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2022 Davide Bettio <davide@uninstall.it>
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

#include <context.h>
#include <stdint.h>

static int draw_image_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item);
static int draw_scaled_cropped_img_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item);
static int draw_rect_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item);
static int draw_text_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item);

static int find_max_line_len(BaseDisplayItem *items, int count, int xpos, int ypos)
{
    int line_len = DISPLAY_WIDTH - xpos;

    for (int i = 0; i < count; i++) {
        BaseDisplayItem *item = &items[i];

        if ((xpos < item->x) && (ypos >= item->y) && (ypos < item->y + item->height)) {
            int len_to_item = item->x - xpos;
            line_len = (line_len > len_to_item) ? len_to_item : line_len;
        }
    }

    return line_len;
}

static int draw_x(uint8_t *line_buf, int xpos, int ypos, BaseDisplayItem *items, int items_count)
{
    bool below = false;

    for (int i = 0; i < items_count; i++) {
        BaseDisplayItem *item = &items[i];
        if ((xpos < item->x) || (xpos >= item->x + item->width) || (ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }

        int max_line_len = below ? 1 : find_max_line_len(items, i, xpos, ypos);

        int drawn_pixels = 0;
        switch (items[i].primitive) {
            case Image:
                //fprintf(stderr, "Image\n");
                drawn_pixels = draw_image_x(line_buf, xpos, ypos, max_line_len, item);
                break;

            case ScaledCroppedImage:
                //fprintf(stderr, "ScaledCroppedImage\n");
                drawn_pixels = draw_scaled_cropped_img_x(line_buf, xpos, ypos, max_line_len, item);
                break;

            case Rect:
                //fprintf(stderr, "Rect\n");
                drawn_pixels = draw_rect_x(line_buf, xpos, ypos, max_line_len, item);
                break;

            case Text:
                //fprintf(stderr, "Text\n");
                drawn_pixels = draw_text_x(line_buf, xpos, ypos, max_line_len, item);
                break;

            default: {
                fprintf(stderr, "unexpected display list command.\n");
            }
        }

        if (drawn_pixels != 0) {
            return drawn_pixels;
        }

        below = true;
    }

    return 1;
}
