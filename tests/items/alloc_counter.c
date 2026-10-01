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

#include <stdlib.h>

long alloc_counter_outstanding;
long alloc_counter_calls;
long alloc_counter_fail_at = -1;

void *alloc_counter_malloc(size_t size)
{
    alloc_counter_calls++;
    if (alloc_counter_calls == alloc_counter_fail_at) {
        return NULL;
    }
    void *ptr = malloc(size);
    if (ptr != NULL) {
        alloc_counter_outstanding++;
    }
    return ptr;
}

void alloc_counter_free(void *ptr)
{
    if (ptr != NULL) {
        alloc_counter_outstanding--;
    }
    free(ptr);
}
