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

#include "spi_display.h"

#include <string.h>

#include <driver/spi_master.h>

#include <globalcontext.h>
#include <interop.h>
#include <term.h>
#include <utils.h>

#include <spi_driver.h>

#include "display_common.h"

bool spi_display_dmawrite(struct SPIDisplay *spi_data, int data_len, const void *data)
{
    memset(&spi_data->transaction, 0, sizeof(spi_transaction_t));

    spi_data->transaction.flags = 0;
    spi_data->transaction.length = data_len * 8;
    spi_data->transaction.addr = 0;
    spi_data->transaction.tx_buffer = data;

    int ret = spi_device_queue_trans(spi_data->handle, &spi_data->transaction, portMAX_DELAY);
    if (UNLIKELY(ret != ESP_OK)) {
        fprintf(stderr, "spidmawrite: transmit error\n");
        return false;
    }

    return true;
}

bool spi_display_write(struct SPIDisplay *spi_data, int data_len, uint32_t data)
{
    memset(&spi_data->transaction, 0, sizeof(spi_transaction_t));

    uint32_t tx_data = SPI_SWAP_DATA_TX(data, data_len);

    spi_data->transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    spi_data->transaction.length = data_len;
    spi_data->transaction.addr = 0;
    spi_data->transaction.tx_data[0] = tx_data;
    spi_data->transaction.tx_data[1] = (tx_data >> 8) & 0xFF;
    spi_data->transaction.tx_data[2] = (tx_data >> 16) & 0xFF;
    spi_data->transaction.tx_data[3] = (tx_data >> 24) & 0xFF;

    // this function is meant for small amount of data so polling is fine here
    int ret = spi_device_polling_transmit(spi_data->handle, &spi_data->transaction);
    if (UNLIKELY(ret != ESP_OK)) {
        fprintf(stderr, "spiwrite: transmit error\n");
        return false;
    }

    return true;
}

bool spi_display_parse_config(struct SPIDisplayConfig *spi_config, term opts, GlobalContext *global)
{
    bool ok = display_common_gpio_from_opts(
        opts, ATOM_STR("\x2", "cs"), &spi_config->cs_gpio, global);

    if (!ok) {
        return false;
    }

    term spi_host_atom = globalcontext_make_atom(global, ATOM_STR("\x8", "spi_host"));
    term spi_port = interop_proplist_get_value(opts, spi_host_atom);

    ok = spi_driver_get_peripheral(spi_port, &spi_config->host_dev, global);

    return ok;
}

bool spi_display_init(struct SPIDisplay *spi_disp, struct SPIDisplayConfig *spi_config)
{
    memset(spi_disp, 0, sizeof(struct SPIDisplay));

    spi_device_interface_config_t devcfg = {
        .mode = spi_config->mode,
        .clock_speed_hz = spi_config->clock_speed_hz,
        .flags = (spi_config->cs_active_high ? SPI_DEVICE_POSITIVE_CS : 0) |
            (spi_config->bit_lsb_first ? SPI_DEVICE_BIT_LSBFIRST : 0),
        .spics_io_num = spi_config->cs_gpio,
        .cs_ena_pretrans = spi_config->cs_ena_pretrans,
        .cs_ena_posttrans = spi_config->cs_ena_posttrans,
        .queue_size = 1
    };

    esp_err_t ret = spi_bus_add_device(spi_config->host_dev, &devcfg, &spi_disp->handle);
    ESP_ERROR_CHECK(ret);

    return true;
}

void spi_display_init_config(struct SPIDisplayConfig *spi_config)
{
    memset(spi_config, 0, sizeof(struct SPIDisplayConfig));
}
