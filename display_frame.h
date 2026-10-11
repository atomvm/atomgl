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

#ifndef _DISPLAY_FRAME_H_
#define _DISPLAY_FRAME_H_

#include <stdbool.h>
#include <stddef.h>

#include <context.h>
#include <mailbox.h>
#include <term.h>

#include "display_damage.h"
#include "display_items.h"

struct DisplayFrame
{
    int screen_width;
    int screen_height;

    // The message owns the term binaries the items point to. Keeping it alive
    // is what makes comparing the items with the next list safe.
    BaseDisplayItem *items;
    size_t items_len;
    Message *message;

    // Areas the next update must repaint regardless of the diff.
    struct DisplayDamage pending;
};

struct DisplayUpdate
{
    BaseDisplayItem *items;
    size_t items_len;
    struct DisplayDamage damage;
};

void display_frame_init(struct DisplayFrame *frame, int screen_width, int screen_height);

// Returns false and leaves the frame untouched when the list is invalid or
// memory is exhausted.
bool display_frame_begin_update(struct DisplayFrame *frame, term display_list, Context *ctx,
    struct DisplayUpdate *update);

// `message`, which owns the binaries of the update's items, is retained until
// the next commit and must not be disposed by the caller. The previous items
// and message are released.
void display_frame_commit(struct DisplayFrame *frame, struct DisplayUpdate *update,
    Message *message, GlobalContext *global);

// The screen content is unknown after a failed or partial draw, so the next
// update repaints everything.
void display_frame_abort(struct DisplayFrame *frame, struct DisplayUpdate *update);

// Gives the memory of the retained list back; the next update repaints
// everything, as the first one does.
void display_frame_release(struct DisplayFrame *frame, GlobalContext *global);

// For screen content changed outside of the display list (e.g. a raw buffer
// was written).
void display_frame_invalidate_rect(struct DisplayFrame *frame, int x, int y, int width, int height);
void display_frame_invalidate_all(struct DisplayFrame *frame);

#endif
