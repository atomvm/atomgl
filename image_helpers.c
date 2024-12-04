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

#include "spng.h"

void handle_load_image(term req, term ref, term pid, Context *ctx)
{
    term image_bin = term_get_tuple_element(req, 1);
    const void *buf = term_binary_data(image_bin);
    size_t buf_size = term_binary_size(image_bin);

    spng_ctx *png_ctx = spng_ctx_new(0);
    spng_set_png_buffer(png_ctx, buf, buf_size);

    size_t out_size;
    spng_decoded_image_size(png_ctx, SPNG_FMT_RGBA8, &out_size);

    void *out = malloc(out_size);
    spng_decode_image(png_ctx, out, out_size, SPNG_FMT_RGBA8, 0);

    spng_ctx_free(png_ctx);

    // term_binary_heap_size(out_size) is usually less than 100 bytes
    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(3) + term_binary_heap_size(out_size), heap);

    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, ref);
    term_put_tuple_element(return_tuple, 1, term_from_literal_binary(out, out_size, &heap, ctx->global));

    free(out);

    int local_process_id = term_to_local_process_id(pid);
    globalcontext_send_message(ctx->global, local_process_id, return_tuple);

    END_WITH_STACK_HEAP(heap, ctx->global)
}
