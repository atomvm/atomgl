/*
 * This file is part of AtomVM.
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

#include <utils.h>

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
