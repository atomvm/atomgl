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

#ifndef _DAMAGE_H_
#define _DAMAGE_H_

#include <stdbool.h>

#include "../display_items.h"

struct Rectangle
{
    int x;
    int y;
    int width;
    int height;
    bool valid;
};

void damage_diff(BaseDisplayItem *orig, int orig_len, BaseDisplayItem *new, int new_len,
    struct Rectangle *damaged);
void damage_clip(struct Rectangle *rectangle, const struct Rectangle *clip_region);

#endif
