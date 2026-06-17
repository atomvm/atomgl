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

#include <esp_idf_version.h>
#include <esp_log.h>

#include <context.h>
#include <interop.h>

#include <esp32_sys.h>

static const char *TAG = "display_driver";

Context *epaper_display_create_port(GlobalContext *global, term opts);
Context *dcs_lcd_display_create_port(GlobalContext *global, term opts);
Context *memory_lcd_display_create_port(GlobalContext *global, term opts);
Context *oled_display_create_port(GlobalContext *global, term opts);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
Context *rgb_lcd_display_create_port(GlobalContext *global, term opts);
#endif

Context *display_create_port(GlobalContext *global, term opts)
{
    term compat_atom = globalcontext_make_atom(global, ATOM_STR("\xA", "compatible"));

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
    if (!strcmp(compat_string, "waveshare,5in65-acep-7c")
        || !strcmp(compat_string, "good-display/gdep073e01")) {
        ctx = epaper_display_create_port(global, opts);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    } else if (!strcmp(compat_string, "waveshare,esp32-s3-touch-lcd-7")
        || !strcmp(compat_string, "esp_lcd,rgb")) {
        ctx = rgb_lcd_display_create_port(global, opts);
#endif
    } else if (!strcmp(compat_string, "sharp,memory-lcd")) {
        ctx = memory_lcd_display_create_port(global, opts);
    } else if (!strcmp(compat_string, "ilitek,ili9341")
        || !strcmp(compat_string, "ilitek,ili9342c")
        || !strcmp(compat_string, "ilitek,ili9486")
        || !strcmp(compat_string, "ilitek,ili9488")
        || !strcmp(compat_string, "sitronix,st7789")
        || !strcmp(compat_string, "sitronix,st7796")) {
        ctx = dcs_lcd_display_create_port(global, opts);
    } else if (!strcmp(compat_string, "solomon-systech,ssd1306")) {
        ctx = oled_display_create_port(global, opts);
    } else if (!strcmp(compat_string, "solomon-systech,ssd1315")) {
        ctx = oled_display_create_port(global, opts);
    } else if (!strcmp(compat_string, "sino-wealth,sh1106")) {
        ctx = oled_display_create_port(global, opts);
    } else {
        ESP_LOGE(TAG, "No matching display driver for given `compatible`: `%s`.", compat_string);
    }

    free(compat_string);

    return ctx;
}

REGISTER_PORT_DRIVER(display, NULL, NULL, display_create_port)
