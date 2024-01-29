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

#include <stdlib.h>

#include <esp_log.h>

#include <context.h>
#include <interop.h>

#include <esp32_sys.h>

static const char *TAG = "display_driver";

Context *acep_5in65_7c_display_driver_create_port(GlobalContext *global, term opts);
Context *ili934x_display_create_port(GlobalContext *global, term opts);
Context *memory_lcd_display_create_port(GlobalContext *global, term opts);
Context *ssd1306_display_create_port(GlobalContext *global, term opts);

Context *display_create_port(GlobalContext *global, term opts)
{
    int compat_atom_index = globalcontext_insert_atom(global, ATOM_STR("\xA", "compatible"));
    term compat_atom = term_from_atom_index(compat_atom_index);

    term compat_value_term = interop_proplist_get_value(opts, compat_atom);
    if (compat_value_term == term_nil()) {
        return NULL;
    }

    int ok;
    char *compat_string = interop_term_to_string(compat_value_term, &ok);

    if (!ok) {
        return NULL;
    }

    Context *ctx = NULL;
    if (!strcmp(compat_string, "waveshare,5in65-acep-7c")) {
        ctx = acep_5in65_7c_display_driver_create_port(global, opts);
    } else if (!strcmp(compat_string, "sharp,memory-lcd")) {
        ctx = memory_lcd_display_create_port(global, opts);
    } else if (!strcmp(compat_string, "ilitek,ili9341")) {
        ctx = ili934x_display_create_port(global, opts);
    } else if (!strcmp(compat_string, "ilitek,ili9342c")) {
        ctx = ili934x_display_create_port(global, opts);
    } else if (!strcmp(compat_string, "solomon-systech,ssd1306")) {
        ctx = ssd1306_display_create_port(global, opts);
    } else {
        ESP_LOGE(TAG, "No matching display driver for given `comptaible`: `%s`.", compat_string);
    }

    free(compat_string);

    return ctx;
}

REGISTER_PORT_DRIVER(display, NULL, NULL, display_create_port)
