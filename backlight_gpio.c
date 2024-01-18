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

#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <term.h>
#include <utils.h>

#include <driver/gpio.h>

#include "display_common.h"

#include "backlight_gpio.h"

enum BacklightActive
{
    ActiveHigh,
    ActiveLow
};

static const AtomStringIntPair backlight_active_table[] = {
    { ATOM_STR("\x4", "high"), ActiveHigh },
    { ATOM_STR("\x3", "low"), ActiveLow },
    SELECT_INT_DEFAULT(-1)
};

void backlight_gpio_init_config(struct BacklightGPIOConfig *backlight_config)
{
    memset(backlight_config, 0, sizeof(struct BacklightGPIOConfig));
}

bool backlight_gpio_parse_config(
    struct BacklightGPIOConfig *backlight_config, term opts, GlobalContext *glb)
{
    bool ok = display_common_gpio_from_opts(
        opts, ATOM_STR("\x9", "backlight"), &backlight_config->gpio, glb);
    if (!ok) {
        backlight_config->configured = false;
        return false;
    }

    term active = interop_kv_get_value_default(
        opts, ATOM_STR("\x10", "backlight_active"), term_invalid_term(), glb);
    if (active != term_invalid_term()) {
        enum BacklightActive backlight_active
            = interop_atom_term_select_int(backlight_active_table, active, glb);
        switch (backlight_active) {
            case ActiveHigh:
                backlight_config->active_high = true;
                break;
            case ActiveLow:
                backlight_config->active_high = false;
                break;
            default:
                return false;
        }
    } else {
        backlight_config->active_high = true;
    }

    term enabled
        = interop_kv_get_value_default(opts, ATOM_STR("\x11", "backlight_enabled"), TRUE_ATOM, glb);
    backlight_config->enabled = enabled == TRUE_ATOM ? true : false;

    backlight_config->configured = true;

    return ok;
}

bool backlight_gpio_init(struct BacklightGPIOConfig *backlight_config)
{
    if (backlight_config->configured) {
        int enabled_level = backlight_config->active_high ? 1 : 0;
        int level = backlight_config->enabled ? enabled_level : !enabled_level;
        gpio_set_direction(backlight_config->gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(backlight_config->gpio, level);
    }

    return true;
}
