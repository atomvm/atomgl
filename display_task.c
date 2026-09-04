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

#include "display_task.h"

#include <defaultatoms.h>
#include <interop.h>
#include <memory.h>
#include <port.h>
#include <term.h>
#include <utils.h>

#include "display_message.h"
#include "ufontlib.h"

UFontManager *ufont_manager;

static bool try_pre_ack_render_cmd(Message *message, Context *ctx)
{
    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message,
                &gen_message) != GenCallMessage)) {
        return false;
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        return false;
    }
    term cmd = term_get_tuple_element(req, 0);

    if (cmd != globalcontext_make_atom(ctx->global, "\x6" "update")
            && cmd != globalcontext_make_atom(ctx->global,
                    "\xB" "draw_buffer")) {
        return false;
    }

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);
    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);

    return true;
}

NativeHandlerResult display_task_consume_mailbox(Context *ctx)
{
    struct DisplayTaskArgs *args = ctx->platform_data;

    MailboxMessage *mbox_msg = mailbox_take_message(&ctx->mailbox);
    Message *msg = CONTAINER_OF(mbox_msg, Message, base);

    // Pre-ack update / draw_buffer before the enqueue-or-drop
    // dance so the caller doesn't time out if dropped.
    try_pre_ack_render_cmd(msg, ctx);

    // Non-blocking enqueue; drop oldest on overflow.
    if (xQueueSend(args->messages_queue, &msg, 0) != pdTRUE) {

        Message *old = NULL;
        if (xQueueReceive(args->messages_queue, &old, 0) == pdTRUE && old) {
            BEGIN_WITH_STACK_HEAP(1, temp_heap);
            mailbox_message_dispose(&old->base, &temp_heap);
            END_WITH_STACK_HEAP(temp_heap, ctx->global);
        }

        if (xQueueSend(args->messages_queue, &msg, 0) != pdTRUE) {
            BEGIN_WITH_STACK_HEAP(1, temp_heap2);
            mailbox_message_dispose(&msg->base, &temp_heap2);
            END_WITH_STACK_HEAP(temp_heap2, ctx->global);
        }
    }

    return NativeContinue;
}

static bool try_handle_register_font(Message *message, Context *ctx)
{
    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message,
                &gen_message) != GenCallMessage)) {
        return false;
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        return false;
    }
    term cmd = term_get_tuple_element(req, 0);

    if (cmd != globalcontext_make_atom(ctx->global,
                "\xD" "register_font")) {
        return false;
    }

    term font_bin = term_get_tuple_element(req, 2);
    size_t font_size = term_binary_size(font_bin);
    void *owned_buf = malloc(font_size);
    EpdFont *loaded_font = NULL;
    if (owned_buf != NULL) {
        memcpy(owned_buf, term_binary_data(font_bin), font_size);
        loaded_font = ufont_parse(owned_buf, font_size);
        if (loaded_font == NULL) {
            free(owned_buf);
        }
    }

    char *handle = interop_atom_to_string(ctx,
            term_get_tuple_element(req, 1));
    if (loaded_font != NULL && handle != NULL) {
        ufont_manager_register(ufont_manager, handle, loaded_font, owned_buf);
    }
    free(handle);

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);
    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);

    return true;
}

static bool try_handle_deregister_font(Message *message, Context *ctx)
{
    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message,
                &gen_message) != GenCallMessage)) {
        return false;
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 2)) {
        return false;
    }
    term cmd = term_get_tuple_element(req, 0);

    if (cmd != globalcontext_make_atom(ctx->global,
                "\xF" "deregister_font")) {
        return false;
    }

    char *handle = interop_atom_to_string(ctx,
            term_get_tuple_element(req, 1));
    if (handle != NULL) {
        ufont_manager_unregister(ufont_manager, handle);
    }
    free(handle);

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);
    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);

    return true;
}

void display_task_process_messages(void *arg)
{
    struct DisplayTaskArgs *args = arg;

    ufont_manager = ufont_manager_new();

    while (true) {
        Message *message;
        xQueueReceive(args->messages_queue, &message, portMAX_DELAY);

        if (!try_handle_register_font(message, args->ctx)
                && !try_handle_deregister_font(message, args->ctx)) {
            args->process_message_fn(message, args->ctx);
        }

        BEGIN_WITH_STACK_HEAP(1, temp_heap);
        mailbox_message_dispose(&message->base, &temp_heap);
        END_WITH_STACK_HEAP(temp_heap, args->ctx->global);
    }
}
