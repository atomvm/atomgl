<!---
  Copyright 2020-2022 Davide Bettio <davide@uninstall.it>

  SPDX-License-Identifier: Apache-2.0
-->

# AtomGL

AtomGL is a display driver for a number of different display models and technologies.

## Supported Hardware

* ILI9341 / ILI9342c: 240x320, 16 bit colors, supported
* Waveshare 7-color 5.65" ACeP display module: 600x480, 7 colors with software dithering, WIP
* Sharp Memory LCDs: 400x240 1-bit monochromatic, WIP

## Notes

This project used to be focused on ILI934x devices, however a lot of boilerplate code can be
shared. Furthermore, this project aims to help creating generic firmwares that support a number of
different display models.
