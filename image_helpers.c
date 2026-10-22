/*
 * This file is part of AtomGL.
 *
 * Copyright 2024 Davide Bettio <davide@uninstall.it>
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

#include "image_helpers.h"

#include <stdio.h>
#include <stdlib.h>

#include <defaultatoms.h>
#include <globalcontext.h>
#include <memory.h>
#include <term.h>
#include <utils.h>

#include "spng.h"

static void send_decoded_image(term ref, term pid, const void *data, size_t size, Context *ctx)
{
    // TODO: change return format to {ok, binary}
    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE + term_binary_heap_size(size), heap);

    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, ref);
    term_put_tuple_element(return_tuple, 1, term_from_literal_binary(data, size, &heap, ctx->global));

    int local_process_id = term_to_local_process_id(pid);
    globalcontext_send_message(ctx->global, local_process_id, return_tuple);

    END_WITH_STACK_HEAP(heap, ctx->global)
}

static void send_load_image_error(term ref, term pid, term reason, Context *ctx)
{
    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + TUPLE_SIZE(2) + REF_SIZE, heap);

    term error_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(error_tuple, 0, ERROR_ATOM);
    term_put_tuple_element(error_tuple, 1, reason);

    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, ref);
    term_put_tuple_element(return_tuple, 1, error_tuple);

    int local_process_id = term_to_local_process_id(pid);
    globalcontext_send_message(ctx->global, local_process_id, return_tuple);

    END_WITH_STACK_HEAP(heap, ctx->global)
}

static term invalid_image_reason(Context *ctx)
{
    term reason = globalcontext_make_atom(ctx->global, "\xD"
                                                       "invalid_image");
    if (UNLIKELY(term_is_invalid_term(reason))) {
        return ERROR_ATOM;
    }
    return reason;
}

void handle_load_image(term req, term ref, term pid, Context *ctx)
{
    if (UNLIKELY(term_get_tuple_arity(req) < 2)) {
        fprintf(stderr, "handle_load_image: missing image argument\n");
        send_load_image_error(ref, pid, BADARG_ATOM, ctx);
        return;
    }

    term image_bin = term_get_tuple_element(req, 1);
    if (UNLIKELY(!term_is_binary(image_bin))) {
        fprintf(stderr, "handle_load_image: image argument is not a binary\n");
        send_load_image_error(ref, pid, BADARG_ATOM, ctx);
        return;
    }

    const void *buf = term_binary_data(image_bin);
    size_t buf_size = term_binary_size(image_bin);

    spng_ctx *png_ctx = spng_ctx_new(0);
    if (UNLIKELY(!png_ctx)) {
        fprintf(stderr, "handle_load_image: spng_ctx_new failed\n");
        send_load_image_error(ref, pid, OUT_OF_MEMORY_ATOM, ctx);
        return;
    }

    int ret = spng_set_png_buffer(png_ctx, buf, buf_size);
    if (UNLIKELY(ret != SPNG_OK)) {
        fprintf(stderr, "handle_load_image: spng_set_png_buffer failed: %s\n", spng_strerror(ret));
        spng_ctx_free(png_ctx);
        send_load_image_error(ref, pid, invalid_image_reason(ctx), ctx);
        return;
    }

    size_t out_size;
    ret = spng_decoded_image_size(png_ctx, SPNG_FMT_RGBA8, &out_size);
    if (UNLIKELY(ret != SPNG_OK)) {
        fprintf(stderr, "handle_load_image: spng_decoded_image_size failed: %s\n", spng_strerror(ret));
        spng_ctx_free(png_ctx);
        send_load_image_error(ref, pid, invalid_image_reason(ctx), ctx);
        return;
    }

    void *out = malloc(out_size);
    if (UNLIKELY(!out)) {
        fprintf(stderr, "handle_load_image: cannot allocate %zu bytes for decoded image\n", out_size);
        spng_ctx_free(png_ctx);
        send_load_image_error(ref, pid, OUT_OF_MEMORY_ATOM, ctx);
        return;
    }

    ret = spng_decode_image(png_ctx, out, out_size, SPNG_FMT_RGBA8, 0);
    spng_ctx_free(png_ctx);
    if (UNLIKELY(ret != SPNG_OK)) {
        fprintf(stderr, "handle_load_image: spng_decode_image failed: %s\n", spng_strerror(ret));
        free(out);
        send_load_image_error(ref, pid, invalid_image_reason(ctx), ctx);
        return;
    }

    send_decoded_image(ref, pid, out, out_size, ctx);

    free(out);
}
