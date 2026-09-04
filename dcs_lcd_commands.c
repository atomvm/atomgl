/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2026 Davide Bettio <davide@uninstall.it>
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

#include "dcs_lcd_commands.h"

#include <stdio.h>
#include <stdlib.h>

#include <utils.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/spi_master.h>
#include <esp_heap_caps.h>

void dcs_lcd_set_paint_area(struct SPIDCBus *bus, const struct DCSLCDScreen *screen,
    int x, int y, int width, int height)
{
    x += screen->x_offset;
    y += screen->y_offset;

    spi_dc_write_command(bus, DCS_LCD_CASET);
    spi_device_acquire_bus(bus->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&bus->spi_disp, 32, (x << 16) | ((x + width) - 1));
    spi_device_release_bus(bus->spi_disp.handle);

    spi_dc_write_command(bus, DCS_LCD_PASET);
    spi_device_acquire_bus(bus->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&bus->spi_disp, 32, (y << 16) | ((y + height) - 1));
    spi_device_release_bus(bus->spi_disp.handle);
}

void dcs_lcd_draw_buffer(struct SPIDCBus *bus, const struct DCSLCDScreen *screen,
    int pixel_bytes, int x, int y, int width, int height, const void *imgdata)
{
    const uint16_t *data = imgdata;

    dcs_lcd_set_paint_area(bus, screen, x, y, width, height);

    spi_dc_write_command(bus, DCS_LCD_RAMWR);

    int dest_size = width * height;
    int chunks = dest_size / 1024;

    spi_device_acquire_bus(bus->spi_disp.handle, portMAX_DELAY);

    if (pixel_bytes == 2) {
        int buf_pixel_size = (dest_size > 1024) ? 1024 : dest_size;
        uint16_t *tmpbuf = heap_caps_malloc(buf_pixel_size * sizeof(uint16_t), MALLOC_CAP_DMA);
        if (UNLIKELY(!tmpbuf)) {
            fprintf(stderr, "dcs_lcd_draw_buffer: failed to alloc tmpbuf (rgb565)\n");
            spi_device_release_bus(bus->spi_disp.handle);
            return;
        }

        for (int i = 0; i < chunks; i++) {
            const uint16_t *data_b = data + 1024 * i;
            for (int j = 0; j < 1024; j++) {
                tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
            }
            spi_display_dma_write(&bus->spi_disp, 1024 * sizeof(uint16_t), tmpbuf);
        }

        int last_chunk_size = dest_size - chunks * 1024;
        if (last_chunk_size) {
            const uint16_t *data_b = data + chunks * 1024;
            for (int j = 0; j < last_chunk_size; j++) {
                tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
            }
            spi_display_dma_write(&bus->spi_disp, last_chunk_size * sizeof(uint16_t), tmpbuf);
        }

        free(tmpbuf);

    } else {
        // ILI9488: RGB565 -> RGB888 (3 bytes/pixel).
        const int chunk_pixels = 512;
        uint8_t *tmpbuf = heap_caps_malloc(chunk_pixels * 3, MALLOC_CAP_DMA);
        if (UNLIKELY(!tmpbuf)) {
            fprintf(stderr, "dcs_lcd_draw_buffer: failed to alloc tmpbuf (rgb888)\n");
            spi_device_release_bus(bus->spi_disp.handle);
            return;
        }

        int i = 0;
        while (i < dest_size) {
            int n = (dest_size - i > chunk_pixels) ? chunk_pixels : (dest_size - i);

            for (int j = 0; j < n; j++) {
                uint16_t px = data[i + j];
                uint8_t r5 = (px >> 11) & 0x1F;
                uint8_t g6 = (px >> 5) & 0x3F;
                uint8_t b5 = (px >> 0) & 0x1F;

                uint8_t r8 = (r5 << 3) | (r5 >> 2);
                uint8_t g8 = (g6 << 2) | (g6 >> 4);
                uint8_t b8 = (b5 << 3) | (b5 >> 2);

                tmpbuf[j * 3 + 0] = r8;
                tmpbuf[j * 3 + 1] = g8;
                tmpbuf[j * 3 + 2] = b8;
            }

            spi_display_dma_write(&bus->spi_disp, n * 3, tmpbuf);
            i += n;
        }

        free(tmpbuf);
    }

    spi_device_release_bus(bus->spi_disp.handle);
}

// --- Init sequence interpreter ---

void dcs_lcd_execute_init_seq(struct SPIDCBus *bus, const uint8_t *seq)
{
    while (*seq != DCS_LCD_INIT_SEQ_END) {
        uint8_t cmd = *seq++;
        uint8_t flags_len = *seq++;
        uint8_t len = flags_len & 0x7F;

        spi_dc_write_cmd_data(bus, cmd, seq, len);
        seq += len;

        if (flags_len & DCS_LCD_INIT_SEQ_DELAY) {
            vTaskDelay(*seq++ / portTICK_PERIOD_MS);
        }
    }
}

// --- Built-in init sequences ---
//
// Each row: CMD, FLAGS_LEN, DATA...  (see dcs_lcd_commands.h for format).
// Transcribed mechanically from the original display_init_*() functions.
// _Static_assert on sizeof guards against miscounted data bytes.

// clang-format off

// ILI9341 — from display_init_9341() in ili934x_display_driver.c
// 21 command groups, 3 delays (SWRESET 5ms, SLPOUT 120ms, DISPON 120ms).
// SLPOUT intentionally follows the vendor power/voltage commands: the
// internal booster must wake up with the configured VMCTR/PWCTR values,
// otherwise many ILI9341 modules never produce usable output.
const uint8_t dcs_lcd_init_seq_ili9341[] = {
    0x01,  DCS_LCD_INIT_SEQ_DELAY | 0,  5,                                             // SWRESET +5ms (0)
    0xEF,  3,  0x03, 0x80, 0x02,                                                       // (3)
    0xCF,  3,  0x00, 0xC1, 0x30,                                                       // Power ctrl B (3)
    0xED,  4,  0x64, 0x03, 0x12, 0x81,                                                 // Power on seq (4)
    0xE8,  3,  0x85, 0x00, 0x78,                                                       // Driver timing A (3)
    0xCB,  5,  0x39, 0x2C, 0x00, 0x34, 0x02,                                           // Power ctrl A (5)
    0xF7,  1,  0x20,                                                                    // Pump ratio (1)
    0xEA,  2,  0x00, 0x00,                                                              // Driver timing B (2)
    0xC0,  1,  0x23,                                                                    // PWCTR1 (1)
    0xC1,  1,  0x10,                                                                    // PWCTR2 (1)
    0xC5,  2,  0x3E, 0x28,                                                              // VMCTR1 (2)
    0xC7,  1,  0x86,                                                                    // VMCTR2 (1)
    0x36,  1,  0x08,                                                                    // MADCTL (1)
    0x3A,  1,  0x55,                                                                    // COLMOD (1)
    0xB1,  2,  0x00, 0x13,                                                              // FRMCTR1 (2)
    0xB6,  3,  0x0A, 0xA2, 0x27,                                                       // DFUNCTR (3)
    0xF2,  1,  0x00,                                                                    // Gamma fn dis (1)
    0x26,  1,  0x01,                                                                    // GAMMASET (1)
    0xE0, 15,  0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,                        // GMCTRP1 (15)
               0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    0xE1, 15,  0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,                        // GMCTRN1 (15)
               0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    0x11,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // SLPOUT +120ms (0)
    0x29,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // DISPON +120ms (0)
    DCS_LCD_INIT_SEQ_END
};
_Static_assert(sizeof(dcs_lcd_init_seq_ili9341) == 113,
    "ili9341 init sequence size mismatch");

// ILI9342C — from display_init_9342c() in ili934x_display_driver.c
// 12 command groups, 3 delays (SWRESET 5ms, SLPOUT 120ms, DISPON 120ms).
// SLPOUT follows the vendor power commands; see ILI9341 seq for details.
const uint8_t dcs_lcd_init_seq_ili9342c[] = {
    0x01,  DCS_LCD_INIT_SEQ_DELAY | 0,  5,                                             // SWRESET +5ms (0)
    0xC8,  3,  0xFF, 0x93, 0x42,                                                       // Vendor enable (3)
    0xC0,  2,  0x12, 0x12,                                                              // PWCTR1 (2)
    0xC1,  1,  0x03,                                                                    // PWCTR2 (1)
    0xB0,  1,  0xE0,                                                                    // (1)
    0xF6,  3,  0x00, 0x01, 0x01,                                                       // Interface ctrl (3)
    0x36,  1,  0xA0,                                                                    // MADCTL MY|MV (1)
    0x3A,  1,  0x55,                                                                    // COLMOD (1)
    0xB6,  3,  0x08, 0x82, 0x27,                                                       // DFUNCTR (3)
    0xE0, 15,  0x00, 0x0C, 0x11, 0x04, 0x11, 0x08, 0x37, 0x89,                        // GMCTRP1 (15)
               0x4C, 0x06, 0x0C, 0x0A, 0x2E, 0x34, 0x0F,
    0xE1, 15,  0x00, 0x0B, 0x11, 0x05, 0x13, 0x09, 0x33, 0x67,                        // GMCTRN1 (15)
               0x48, 0x07, 0x0E, 0x0B, 0x2E, 0x33, 0x0F,
    0x11,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // SLPOUT +120ms (0)
    0x29,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // DISPON +120ms (0)
    DCS_LCD_INIT_SEQ_END
};
_Static_assert(sizeof(dcs_lcd_init_seq_ili9342c) == 75,
    "ili9342c init sequence size mismatch");

// ILI9486 — from display_init_9486() in ili948x_display_driver.c
// 9 command groups, 3 delays (SWRESET 5ms, SLPOUT 120ms, DISPON 120ms).
// SLPOUT follows the vendor power commands; see ILI9341 seq for details.
const uint8_t dcs_lcd_init_seq_ili9486[] = {
    0x01,  DCS_LCD_INIT_SEQ_DELAY | 0,  5,                                             // SWRESET +5ms (0)
    0xB0,  1,  0x00,                                                                    // IFMODE (1)
    0x3A,  1,  0x55,                                                                    // COLMOD (1)
    0xC2,  1,  0x44,                                                                    // PWRCTR3 (1)
    0xC5,  4,  0x00, 0x00, 0x00, 0x00,                                                 // VMCTR1 (4)
    0xE0, 15,  0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98,                        // PGAMCTRL (15)
               0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00,
    0xE1, 15,  0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75,                        // NGAMCTRL (15)
               0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
    0xE2, 15,  0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75,                        // DGAMCTRL (15)
               0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
    0x11,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // SLPOUT +120ms (0)
    0x29,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // DISPON +120ms (0)
    DCS_LCD_INIT_SEQ_END
};
_Static_assert(sizeof(dcs_lcd_init_seq_ili9486) == 76,
    "ili9486 init sequence size mismatch");

// ILI9488 — from display_init_9488() in ili948x_display_driver.c
// 16 command groups, 3 delays (SWRESET 5ms, SLPOUT 120ms, DISPON 120ms).
// RGB666 (COLMOD 0x66, 3 bytes/pixel).
// SLPOUT follows the vendor power commands; see ILI9341 seq for details.
const uint8_t dcs_lcd_init_seq_ili9488[] = {
    0x01,  DCS_LCD_INIT_SEQ_DELAY | 0,  5,                                             // SWRESET +5ms (0)
    0xB0,  1,  0x00,                                                                    // IFMODE (1)
    0xF7,  4,  0xA9, 0x51, 0x2C, 0x82,                                                 // ADJCTRL3 (4)
    0xC0,  2,  0x11, 0x09,                                                              // PWRCTR1 (2)
    0xC1,  1,  0x41,                                                                    // PWRCTR2 (1)
    0xC5,  3,  0x00, 0x0A, 0x80,                                                       // VMCTR1 (3)
    0xB1,  2,  0xB0, 0x11,                                                              // FRMCTR1 (2)
    0xB4,  1,  0x02,                                                                    // INVCTR (1)
    0xB6,  2,  0x02, 0x02,                                                              // DFUNCTR (2)
    0xB7,  1,  0xC6,                                                                    // ETMOD (1)
    0xBE,  2,  0x00, 0x04,                                                              // HS lanes ctrl (2)
    0xE9,  1,  0x00,                                                                    // Image fn (1)
    0x3A,  1,  0x66,                                                                    // COLMOD RGB666 (1)
    0xE0, 15,  0x00, 0x07, 0x10, 0x09, 0x17, 0x0B, 0x41, 0x89,                        // PGAMCTRL (15)
               0x4B, 0x0A, 0x0C, 0x0E, 0x18, 0x1B, 0x0F,
    0xE1, 15,  0x00, 0x17, 0x1A, 0x04, 0x0E, 0x06, 0x2F, 0x45,                        // NGAMCTRL (15)
               0x43, 0x02, 0x0A, 0x09, 0x32, 0x36, 0x0F,
    0x11,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // SLPOUT +120ms (0)
    0x29,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // DISPON +120ms (0)
    DCS_LCD_INIT_SEQ_END
};
_Static_assert(sizeof(dcs_lcd_init_seq_ili9488) == 89,
    "ili9488 init sequence size mismatch");

// ST7789 standard — from display_init_std() in st7789_display_driver.c
// 21 command groups, 4 delays (SWRESET 5ms, SLPOUT 120ms, COLMOD 10ms,
// DISPON 120ms).
const uint8_t dcs_lcd_init_seq_st7789_std[] = {
    0x01,  DCS_LCD_INIT_SEQ_DELAY | 0,  5,                                             // SWRESET +5ms (0)
    0x11,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // SLPOUT +120ms (0)
    0x13,  0,                                                                           // NORON (0)
    0x36,  1,  0x00,                                                                    // MADCTL (1)
    0xB6,  2,  0x0A, 0x82,                                                              // (2)
    0xB0,  2,  0x00, 0xE0,                                                              // RAMCTRL (2)
    0x3A,  DCS_LCD_INIT_SEQ_DELAY | 1,  0x55,  10,                                     // COLMOD +10ms (1)
    0xB2,  5,  0x0C, 0x0C, 0x00, 0x33, 0x33,                                           // PORCTRL (5)
    0xB7,  1,  0x35,                                                                    // GCTRL (1)
    0xBB,  1,  0x28,                                                                    // VCOMS (1)
    0xC0,  1,  0x0C,                                                                    // LCMCTRL (1)
    0xC2,  2,  0x01, 0xFF,                                                              // VDVVRHEN (2)
    0xC3,  1,  0x10,                                                                    // VRHS (1)
    0xC4,  1,  0x20,                                                                    // VDVSET (1)
    0xC6,  1,  0x0F,                                                                    // FRCTR2 (1)
    0xD0,  2,  0xA4, 0xA1,                                                              // PWCTRL1 (2)
    0xE0, 14,  0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x32, 0x44,                        // PVGAMCTRL (14)
               0x42, 0x06, 0x0E, 0x12, 0x14, 0x17,
    0xE1, 14,  0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x31, 0x54,                        // NVGAMCTRL (14)
               0x47, 0x0E, 0x1C, 0x17, 0x1B, 0x1E,
    0x2A,  4,  0x00, 0x00, 0x00, 0xEF,                                                 // CASET 0-239 (4)
    0x2B,  4,  0x00, 0x00, 0x01, 0x3F,                                                 // PASET 0-319 (4)
    0x29,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // DISPON +120ms (0)
    DCS_LCD_INIT_SEQ_END
};
_Static_assert(sizeof(dcs_lcd_init_seq_st7789_std) == 104,
    "st7789 std init sequence size mismatch");

// ST7789 alt gamma 2 — from display_init_alt_gamma_2() in st7789_display_driver.c
// 19 command groups, 4 delays (SWRESET 5ms, SLPOUT 120ms, COLMOD 10ms,
// DISPON 120ms).
const uint8_t dcs_lcd_init_seq_st7789_alt[] = {
    0x01,  DCS_LCD_INIT_SEQ_DELAY | 0,  5,                                             // SWRESET +5ms (0)
    0x11,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // SLPOUT +120ms (0)
    0x13,  0,                                                                           // NORON (0)
    0x36,  1,  0x00,                                                                    // MADCTL (1)
    0x3A,  DCS_LCD_INIT_SEQ_DELAY | 1,  0x55,  10,                                     // COLMOD +10ms (1)
    0xB2,  5,  0x0C, 0x0C, 0x00, 0x33, 0x33,                                           // PORCTRL (5)
    0xB7,  1,  0x75,                                                                    // GCTRL (1)
    0xBB,  1,  0x1A,                                                                    // VCOMS (1)
    0xC0,  1,  0x2C,                                                                    // LCMCTRL (1)
    0xC2,  1,  0x01,                                                                    // VDVVRHEN (1)
    0xC3,  1,  0x13,                                                                    // VRHS (1)
    0xC4,  1,  0x20,                                                                    // VDVSET (1)
    0xC6,  1,  0x0F,                                                                    // FRCTR2 (1)
    0xD0,  2,  0xA4, 0xA1,                                                              // PWCTRL1 (2)
    0xE0, 14,  0xD0, 0x0D, 0x14, 0x0D, 0x0D, 0x09, 0x38, 0x44,                        // PVGAMCTRL (14)
               0x4E, 0x3A, 0x17, 0x18, 0x2F, 0x30,
    0xE1, 14,  0xD0, 0x09, 0x0F, 0x08, 0x07, 0x14, 0x37, 0x44,                        // NVGAMCTRL (14)
               0x4D, 0x38, 0x15, 0x16, 0x2C, 0x3E,
    0x2A,  4,  0x00, 0x00, 0x00, 0xEF,                                                 // CASET 0-239 (4)
    0x2B,  4,  0x00, 0x00, 0x01, 0x3F,                                                 // PASET 0-319 (4)
    0x29,  DCS_LCD_INIT_SEQ_DELAY | 0,  120,                                           // DISPON +120ms (0)
    DCS_LCD_INIT_SEQ_END
};
_Static_assert(sizeof(dcs_lcd_init_seq_st7789_alt) == 95,
    "st7789 alt gamma 2 init sequence size mismatch");

// clang-format on

// --- Per-controller descriptors ---
//
// madctl[4] values come from the current set_rotation() behavior:
//   0xFF marks a rotation that the existing driver did not validate.
// ILI948x has all 4 rotations (madctl_bgr ORed in at runtime from driver
// state, so the table stores the base value without BGR).
// ILI934x / ST7789 only support rotation 0 (init-default MADCTL) and
// rotation 1 (overridden by set_rotation).

const struct DCSLCDDesc dcs_lcd_desc_ili9341 = {
    .name              = "ILI9341",
    .native_width      = 240,
    .native_height     = 320,
    .spi_clock_hz      = 27000000,
    .pixel_bytes       = 2,
    .colmod_value      = 0x55,
    .madctl            = { 0x08, 0xA8, 0xFF, 0xFF },
    .default_bgr       = true,
    .default_init_seq  = dcs_lcd_init_seq_ili9341,
    .init_fn           = NULL,
};

const struct DCSLCDDesc dcs_lcd_desc_ili9342c = {
    .name              = "ILI9342C",
    .native_width      = 320,
    .native_height     = 240,
    .spi_clock_hz      = 27000000,
    .pixel_bytes       = 2,
    .colmod_value      = 0x55,
    .madctl            = { 0xA0, 0xA8, 0xFF, 0xFF },
    .default_bgr       = false,
    .default_init_seq  = dcs_lcd_init_seq_ili9342c,
    .init_fn           = NULL,
};

const struct DCSLCDDesc dcs_lcd_desc_ili9486 = {
    .name              = "ILI9486",
    .native_width      = 320,
    .native_height     = 480,
    .spi_clock_hz      = 27000000,
    .pixel_bytes       = 2,
    .colmod_value      = 0x55,
    .madctl            = { 0x40, 0x20, 0x80, 0xE0 },
    .default_bgr       = true,
    .default_init_seq  = dcs_lcd_init_seq_ili9486,
    .init_fn           = NULL,
};

const struct DCSLCDDesc dcs_lcd_desc_ili9488 = {
    .name              = "ILI9488",
    .native_width      = 320,
    .native_height     = 480,
    .spi_clock_hz      = 27000000,
    .pixel_bytes       = 3,
    .colmod_value      = 0x66,
    .madctl            = { 0x40, 0x20, 0x80, 0xE0 },
    .default_bgr       = true,
    .default_init_seq  = dcs_lcd_init_seq_ili9488,
    .init_fn           = NULL,
};

const struct DCSLCDDesc dcs_lcd_desc_st7789 = {
    .name              = "ST7789",
    .native_width      = 240,
    .native_height     = 320,
    .spi_clock_hz      = 40000000,
    .pixel_bytes       = 2,
    .colmod_value      = 0x55,
    .madctl            = { 0x00, 0x60, 0xC0, 0xA0 },
    .default_bgr       = false,
    .default_init_seq  = dcs_lcd_init_seq_st7789_std,
    .init_fn           = NULL,
};

// ST7796 currently routes through the ST7789 driver and uses the same
// init sequence.  It may diverge in a future session.
const struct DCSLCDDesc dcs_lcd_desc_st7796 = {
    .name              = "ST7796",
    .native_width      = 320,
    .native_height     = 480,
    .spi_clock_hz      = 40000000,
    .pixel_bytes       = 2,
    .colmod_value      = 0x55,
    .madctl            = { 0x00, 0x60, 0xFF, 0xFF },
    .default_bgr       = false,
    .default_init_seq  = dcs_lcd_init_seq_st7789_std,
    .init_fn           = NULL,
};
