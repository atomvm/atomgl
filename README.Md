<!---
  Copyright 2020-2024 Davide Bettio <davide@uninstall.it>

  SPDX-License-Identifier: Apache-2.0
-->

# AtomGL

AtomGL is a display driver for a number of different display models and technologies.

This component is not meant for being used directly, instead an additional layer should take care of
pushing updates to it. For this reason no additional NIFs are provided.

**Please, use this component with a supported AtomVM version, such as v0.6.**

## Supported Hardware

* `ilitek,ili9341` / `ilitek,ili9342c`: ILI9341 / ILI9342c: 240x320, 16 bit colors, supported
* `waveshare,5in65-acep-7c`: Waveshare 7-color 5.65" ACeP display module: 600x480, 7 colors +
software dithering
* `sharp,memory-lcd`: Sharp Memory LCDs: 400x240 1-bit monochromatic
* `solomon-systech,ssd1306`: Solomon Systech SSD1306: 128x64 1-bit monochromatic

[SDL Linux display](sdl_display/) is also supported and can be built as AtomVM plugin.

## Getting Started

1. Add this component to the ESP32 components directory
2. Open a display port using the right options
3. Start an avm_scene that will push updates to the display

### Display Option Example

The following is an example for ILI9341:

```elixir
  [...]
  # spi is already open

  ili_display_opts = [
    width: 320,
    height: 240,
    compatible: "ilitek,ili9341",
    reset: 18,
    cs: 22,
    dc: 21,
    backlight: 5,
    backlight_active: :low,
    backlight_enabled: true,
    rotation: 1,
    enable_tft_invon: false,
    spi_host: spi
  ]

  :erlang.open_port({:spawn, "display"}, display_opts)
  [...]
```

## Primitives

The display driver takes care of drawing a list of primitive items. Such as:

```elixir
    rendered = [
      {:text, 10, 20, :default16px, 0x000000, 0x808080, "Hello."},
      {:rect, 0, 0, width, height, 0x808080}
    ]
```

Following primitives are supported:
* image
* scaled_cropped_image
* rect
* text
