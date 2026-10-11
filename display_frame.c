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

#include "display_frame.h"

#include <mailbox.h>
#include <memory.h>
#include <term.h>
#include <utils.h>

#include "display_damage.h"
#include "display_diff.h"
#include "display_items.h"

void display_frame_init(struct DisplayFrame *frame, int screen_width, int screen_height)
{
    frame->screen_width = screen_width;
    frame->screen_height = screen_height;
    frame->items = NULL;
    frame->items_len = 0;
    frame->message = NULL;
    display_damage_init(&frame->pending, screen_width, screen_height);
    display_damage_add_all(&frame->pending);
}

bool display_frame_begin_update(struct DisplayFrame *frame, term display_list, Context *ctx,
    struct DisplayUpdate *update)
{
    BaseDisplayItem *items;
    size_t items_len;
    if (UNLIKELY(display_items_new_list(display_list, &items, &items_len, ctx) != DisplayItemsOk)) {
        return false;
    }

    update->items = items;
    update->items_len = items_len;
    display_damage_init(&update->damage, frame->screen_width, frame->screen_height);
    display_damage_add_damage(&update->damage, &frame->pending);
    if (!display_damage_is_full(&update->damage)) {
        display_diff_compute(frame->items, frame->items_len, items, items_len, &update->damage);
    }

    return true;
}

static void dispose_message(Message *message, GlobalContext *global)
{
    BEGIN_WITH_STACK_HEAP(1, temp_heap);
    mailbox_message_dispose(&message->base, &temp_heap);
    END_WITH_STACK_HEAP(temp_heap, global);
}

static void delete_content(struct DisplayFrame *frame, GlobalContext *global)
{
    display_items_delete(frame->items, frame->items_len);
    frame->items = NULL;
    frame->items_len = 0;
    if (frame->message != NULL) {
        dispose_message(frame->message, global);
        frame->message = NULL;
    }
}

void display_frame_commit(struct DisplayFrame *frame, struct DisplayUpdate *update,
    Message *message, GlobalContext *global)
{
    delete_content(frame, global);

    frame->items = update->items;
    frame->items_len = update->items_len;
    frame->message = message;
    display_damage_clear(&frame->pending);

    update->items = NULL;
    update->items_len = 0;
}

void display_frame_abort(struct DisplayFrame *frame, struct DisplayUpdate *update)
{
    display_items_delete(update->items, update->items_len);
    update->items = NULL;
    update->items_len = 0;

    display_damage_add_all(&frame->pending);
}

void display_frame_release(struct DisplayFrame *frame, GlobalContext *global)
{
    delete_content(frame, global);
    display_damage_add_all(&frame->pending);
}

void display_frame_invalidate_rect(struct DisplayFrame *frame, int x, int y, int width, int height)
{
    display_damage_add_rect(&frame->pending, x, y, width, height);
}

void display_frame_invalidate_all(struct DisplayFrame *frame)
{
    display_damage_add_all(&frame->pending);
}
