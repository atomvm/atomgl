/*
 * This file is part of AtomGL.
 *
 * Copyright 2022 Davide Bettio <davide@uninstall.it>
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

#include "display_common.h"

#include <interop.h>

bool display_common_gpio_from_opts(
    term opts, const char *atom_str, int *gpio_num, GlobalContext *global)
{
    int atom_index = globalcontext_insert_atom(global, atom_str);
    term gpio_atom = term_from_atom_index(atom_index);

    term gpio_term = interop_proplist_get_value(opts, gpio_atom);
    if (gpio_term == term_nil()) {
        return false;
    }

    *gpio_num = term_to_int(gpio_term);

    return true;
}
