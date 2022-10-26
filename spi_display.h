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

#ifndef _SPI_DISPLAY_H_
#define _SPI_DISPLAY_H_

#include <driver/spi_master.h>

#include <stdbool.h>

#include <globalcontext.h>

struct SPIDisplay
{
    spi_device_handle_t handle;
    spi_transaction_t transaction;
};

struct SPIDisplayConfig
{
    spi_host_device_t host_dev;
    int cs_gpio;
    int mode;
    int clock_speed_hz;
    bool cs_active_high : 1;
    bool bit_lsb_first : 1;
    int cs_ena_pretrans;
    int cs_ena_posttrans;
};

bool spi_display_init(struct SPIDisplay *spi_disp, struct SPIDisplayConfig *spi_config);
bool spi_display_dmawrite(struct SPIDisplay *spi_data, int data_len, const void *data);
bool spi_display_write(struct SPIDisplay *spi_data, int data_len, uint32_t data);
void spi_display_init_config(struct SPIDisplayConfig *spi_config);
bool spi_display_parse_config(struct SPIDisplayConfig *spi_config, term opts, GlobalContext *global);

void spi_display_bus_init();

#endif
