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

#ifndef _ALLOC_COUNTER_H_
#define _ALLOC_COUNTER_H_

#include <stdlib.h>

extern long alloc_counter_outstanding;
extern long alloc_counter_calls;
extern long alloc_counter_fail_at;

void *alloc_counter_malloc(size_t size);
void alloc_counter_free(void *ptr);

#define malloc(size) alloc_counter_malloc(size)
#define free(ptr) alloc_counter_free(ptr)

#endif
