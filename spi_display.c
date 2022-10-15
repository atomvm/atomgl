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

#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/spi_master.h>

#include <globalcontext.h>
#include <interop.h>
#include <term.h>
#include <utils.h>

#define MISO_IO_NUM CONFIG_AVM_DISPLAY_MISO_IO_NUM
#define MOSI_IO_NUM CONFIG_AVM_DISPLAY_MOSI_IO_NUM
#define SCLK_IO_NUM CONFIG_AVM_DISPLAY_SCLK_IO_NUM

#define SD_ENABLE CONFIG_AVM_DISPLAY_SD_ENABLE
#define SD_CS_IO_NUM CONFIG_AVM_DISPLAY_SD_CS_IO_NUM

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
    int spi_cs_gpio_atom_index = globalcontext_insert_atom(global, ATOM_STR("\xB", "spi_cs_gpio"));
    term spi_cs_gpio_atom = term_from_atom_index(spi_cs_gpio_atom_index);

    term cs_gpio_term = interop_proplist_get_value(opts, spi_cs_gpio_atom);
    if (cs_gpio_term == term_nil()) {
        return false;
    }

    spi_config->cs_gpio = term_to_int(cs_gpio_term);

    return true;
}

bool spi_display_init(struct SPIDisplay *spi_disp, struct SPIDisplayConfig *spi_config)
{
    memset(spi_disp, 0, sizeof(struct SPIDisplay));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = spi_config->cs_gpio,
        .queue_size = 1
    };

    esp_err_t ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_disp->handle);
    ESP_ERROR_CHECK(ret);

    return true;
}

void spi_display_init_config(struct SPIDisplayConfig *spi_config)
{
    memset(spi_config, 0, sizeof(struct SPIDisplayConfig));
}

#if SD_ENABLE == true
// sdcard init in display driver is quite an odd choice
// however display and SD share the same bus
static bool sdcard_init()
{
    ESP_LOGI("sdcard", "Trying SD init.");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = MISO_IO_NUM;
    slot_config.gpio_mosi = MOSI_IO_NUM;
    slot_config.gpio_sck = SCLK_IO_NUM;
    slot_config.gpio_cs = SD_CS_IO_NUM;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("sdcard", "Failed to mount filesystem.");
        } else {
            ESP_LOGE("sdcard", "Failed to initialize the card (%s).", esp_err_to_name(ret));
        }
        return false;
    }

    sdmmc_card_print_info(stdout, card);

    return true;
}
#endif

void spi_display_bus_init()
{
#if SD_ENABLE == true
    if (!sdcard_init()) {
#endif
        spi_bus_config_t buscfg = {
            .miso_io_num = MISO_IO_NUM,
            .mosi_io_num = MOSI_IO_NUM,
            .sclk_io_num = SCLK_IO_NUM,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };

        esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
        ESP_ERROR_CHECK(ret);
#if SD_ENABLE == true
    }
#endif
}
