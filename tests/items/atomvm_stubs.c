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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <atom.h>
#include <atom_table.h>
#include <context.h>
#include <interop.h>
#include <memory.h>
#include <refc_binary.h>
#include <term.h>

#include "alloc_counter.h"

#define FIRST_ATOM_INDEX 256
#define MAX_ATOMS 64

static char *atoms[MAX_ATOMS];
static int atoms_len;

enum AtomTableEnsureAtomResult atom_table_ensure_atom(struct AtomTable *table,
    const uint8_t *atom_data, size_t atom_len, enum AtomTableCopyOpt opts, atom_index_t *result)
{
    (void) table;
    (void) opts;

    for (int i = 0; i < atoms_len; i++) {
        if (strlen(atoms[i]) == atom_len && memcmp(atoms[i], atom_data, atom_len) == 0) {
            *result = FIRST_ATOM_INDEX + i;
            return AtomTableEnsureAtomOk;
        }
    }
    if (atoms_len == MAX_ATOMS) {
        abort();
    }
    char *copy = calloc(1, atom_len + 1);
    if (copy == NULL) {
        abort();
    }
    memcpy(copy, atom_data, atom_len);
    atoms[atoms_len] = copy;
    *result = FIRST_ATOM_INDEX + atoms_len++;

    return AtomTableEnsureAtomOk;
}

int term_display_non_atoms;

void term_display(FILE *fd, term t, const Context *ctx)
{
    (void) ctx;
    if (term_is_atom(t)) {
        int index = term_to_atom_index(t) - FIRST_ATOM_INDEX;
        if (index >= 0 && index < atoms_len) {
            fprintf(fd, "%s", atoms[index]);
            return;
        }
    } else {
        term_display_non_atoms++;
    }
    fprintf(fd, "<term %#llx>", (unsigned long long) t);
}

char *interop_term_to_string(term t, int *ok)
{
    *ok = 0;
    if (term_is_binary(t)) {
        size_t len = term_binary_size(t);
        char *str = malloc(len + 1);
        if (str == NULL) {
            return NULL;
        }
        memcpy(str, term_binary_data(t), len);
        str[len] = 0;
        *ok = 1;
        return str;
    }
    int proper;
    int len = term_list_length(t, &proper);
    if (!proper) {
        return NULL;
    }
    char *str = malloc(len + 1);
    if (str == NULL) {
        return NULL;
    }
    for (int i = 0; i < len; i++) {
        term c = term_get_list_head(t);
        if (!term_is_integer(c) || term_to_int(c) < 0 || term_to_int(c) > 255) {
            free(str);
            return NULL;
        }
        str[i] = (char) term_to_int(c);
        t = term_get_list_tail(t);
    }
    str[len] = 0;
    *ok = 1;
    return str;
}

const char *refc_binary_get_data(const struct RefcBinary *ptr)
{
    (void) ptr;
    abort();
}

term term_alloc_refc_binary(size_t size, bool is_const, Heap *heap, GlobalContext *glb)
{
    (void) glb;
    if (!is_const) {
        abort();
    }
    term *boxed_value = memory_heap_alloc(heap, TERM_BOXED_REFC_BINARY_SIZE);
    boxed_value[0] = ((TERM_BOXED_REFC_BINARY_SIZE - 1) << 6) | TERM_BOXED_REFC_BINARY;
    boxed_value[1] = (term) size;
    boxed_value[2] = (term) RefcBinaryIsConst;
    boxed_value[3] = (term) NULL;
    boxed_value[4] = term_nil();
    boxed_value[5] = term_nil();
    return ((term) boxed_value) | TERM_PRIMARY_BOXED;
}
