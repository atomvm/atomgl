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

#ifndef _DCS_LCD_COMMANDS_H_
#define _DCS_LCD_COMMANDS_H_

#include <stdbool.h>
#include <stddef.h>

#include "dcs_lcd_screen.h"
#include "spi_dc_driver.h"

// Generic MIPI DCS command set (subset used by AtomGL DCS LCD drivers).
// Vendor-specific init sequence bytes (ILI9341_FRMCTR1, ST7789_PORCTRL,
// ILI948X_HS_LANES_CTRL, etc.) stay in each driver's header block.
#define DCS_LCD_SWRESET 0x01
#define DCS_LCD_SLPIN   0x10
#define DCS_LCD_SLPOUT  0x11
#define DCS_LCD_NORON   0x13
#define DCS_LCD_INVOFF  0x20
#define DCS_LCD_INVON   0x21
#define DCS_LCD_DISPOFF 0x28
#define DCS_LCD_DISPON  0x29
#define DCS_LCD_CASET   0x2A
#define DCS_LCD_PASET   0x2B
#define DCS_LCD_RAMWR   0x2C
#define DCS_LCD_MADCTL  0x36
#define DCS_LCD_COLMOD  0x3A

// MADCTL bit positions.
#define DCS_LCD_MAD_MY  0x80
#define DCS_LCD_MAD_MX  0x40
#define DCS_LCD_MAD_MV  0x20
#define DCS_LCD_MAD_ML  0x10
#define DCS_LCD_MAD_BGR 0x08

void dcs_lcd_set_paint_area(struct SPIDCBus *bus, const struct DCSLCDScreen *screen,
    int x, int y, int width, int height);

void dcs_lcd_draw_buffer(struct SPIDCBus *bus, const struct DCSLCDScreen *screen,
    int pixel_bytes, int x, int y, int width, int height, const void *imgdata);

// --- Init sequence byte-array format ---
//
// Each entry:  [CMD] [FLAGS_LEN] [DATA_0 ... DATA_N] [DELAY_MS]
//   CMD:        command byte (0x01-0xFF)
//   FLAGS_LEN:  bits 6:0 = data byte count (0-127)
//               bit 7    = delay flag (DELAY_MS byte follows data)
//   DELAY_MS:   delay in milliseconds (0-255), present only if flag set
//
// End marker: single DCS_LCD_INIT_SEQ_END (0x00) byte.

#define DCS_LCD_INIT_SEQ_END   0x00
#define DCS_LCD_INIT_SEQ_DELAY 0x80

void dcs_lcd_execute_init_seq(struct SPIDCBus *bus, const uint8_t *seq);

// Built-in init sequences.
extern const uint8_t dcs_lcd_init_seq_ili9341[];
extern const uint8_t dcs_lcd_init_seq_ili9341v[];
extern const uint8_t dcs_lcd_init_seq_ili9342c[];
extern const uint8_t dcs_lcd_init_seq_ili9486[];
extern const uint8_t dcs_lcd_init_seq_ili9488[];
extern const uint8_t dcs_lcd_init_seq_st7789_std[];
extern const uint8_t dcs_lcd_init_seq_st7789_alt[];

// --- Per-controller descriptor ---
//
// Describes a DCS LCD controller variant so a unified driver can dispatch
// on compatible string.  madctl[4] holds per-rotation MADCTL values (0xFF
// for rotations not validated on the current hardware).  init_fn is a
// fallback when a byte-array init sequence is insufficient; NULL for all
// current controllers.

struct DCSLCDDriver;  // forward declaration for init_fn

struct DCSLCDDesc
{
    const char *name;
    int native_width;
    int native_height;
    int spi_clock_hz;
    uint8_t pixel_bytes;
    uint8_t colmod_value;
    uint8_t madctl[4];
    bool default_bgr;
    const uint8_t *default_init_seq;
    void (*init_fn)(struct DCSLCDDriver *);
};

extern const struct DCSLCDDesc dcs_lcd_desc_ili9341;
extern const struct DCSLCDDesc dcs_lcd_desc_ili9341v;
extern const struct DCSLCDDesc dcs_lcd_desc_ili9342c;
extern const struct DCSLCDDesc dcs_lcd_desc_ili9486;
extern const struct DCSLCDDesc dcs_lcd_desc_ili9488;
extern const struct DCSLCDDesc dcs_lcd_desc_st7789;
extern const struct DCSLCDDesc dcs_lcd_desc_st7796;

#endif
