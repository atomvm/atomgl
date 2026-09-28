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

#ifndef _DISPLAY_DIFF_H_
#define _DISPLAY_DIFF_H_

#include <stdbool.h>
#include <stddef.h>

#include "display_damage.h"
#include "display_items.h"

// True when the two items draw exactly the same pixels. Images must share the
// pixel buffer: their content is never compared, to keep updates quick.
bool display_diff_items_equal(const BaseDisplayItem *a, const BaseDisplayItem *b);

// Adds to `damage` every area whose content may differ between the display
// list currently shown (`old_items`) and the one about to be shown
// (`new_items`). Outside the damage, a pixel is covered by the same items in
// the same order in both lists, hence keeps its color.
void display_diff_compute(const BaseDisplayItem old_items[], size_t old_items_len,
    const BaseDisplayItem new_items[], size_t new_items_len, struct DisplayDamage *damage);

#endif
