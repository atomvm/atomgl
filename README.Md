<!---
  Copyright 2020-2024 Davide Bettio <davide@uninstall.it>

  SPDX-License-Identifier: Apache-2.0
-->

# AtomGL

AtomGL is a display driver for various display models and technologies, implementing a declarative
display list approach for rendering.

This component is not meant to be used directly. Instead, an additional layer should handle pushing
updates to it. For this reason, no additional NIFs are provided.

**Please use this component with a supported AtomVM version, such as v0.6.**

## Core Concept: Display Lists

AtomGL uses a **declarative display list** approach where everything rendered on the display is
represented as a list of primitive items (Erlang tuples/records). Unlike imperative graphics APIs
where you issue drawing commands sequentially, AtomGL replaces the entire display content with a new
list on each update.

### Z-Ordering

Items in the display list follow a specific rendering order:
- Items are rendered from **tail to head** (reverse order)
- The **first item** in the list has the highest z-order (appears on top)
- The **last item** has the lowest z-order (appears at the bottom)

```elixir
# Example: "Hello" text over a gray background
items = [
  {:text, 10, 20, :default16px, 0x000000, 0x808080, "Hello."},  # Drawn last (on top)
  {:rect, 0, 0, width, height, 0x808080}                        # Drawn first (background)
]
```

**Best Practice:** Always include a full-screen rectangle as the last item in your list to clear the
screen with a background color.

### Declarative vs. Imperative

AtomGL's declarative approach means:
- Each `update` call **completely replaces** the previous display list
- There is no hidden global display state
- You describe *what* should be displayed, not *how* to draw it
- Perfect fit for functional programming paradigms

```erlang
% First update: show background
BackgroundScene = [
    {rect, 0, 0, 1024, 600, 16#FFFFFF}
],
ok = port:call(Display, {update, BackgroundScene}),

% Second update: completely replaces the previous scene
ColoredRectScene = [
    {rect, 150, 150, 80, 80, 16#FF0000},     % Red square (top)
    {rect, 250, 150, 80, 80, 16#00FF00},     % Green square
    {rect, 350, 150, 80, 80, 16#0000FF},     % Blue square
    {rect, 0, 0, 1024, 600, 16#FFFFFF}       % White background (bottom)
],
ok = port:call(Display, {update, ColoredRectScene})
```

Note: Without a background rectangle, some drivers may retain the previous image, but this is not a
guaranteed feature.

### Memory Efficiency

A key benefit of the display list approach is **memory efficiency**. Unlike traditional graphics
systems that require a complete frame buffer in memory, AtomGL drivers can:
- Draw everything in smaller chunks or line by line
- Evaluate display content pixel by pixel on-the-fly
- Drive displays that would require more frame buffer memory than available internal RAM

This makes it possible for microcontrollers with limited memory to drive high-resolution displays
that would otherwise be impossible to support with traditional frame buffer approaches.

## Supported Hardware

* `ilitek,ili9341` / `ilitek,ili9342c`: ILI9341 / ILI9342C - 240x320, 16-bit colors
* `waveshare,5in65-acep-7c`: Waveshare 7-color 5.65" ACeP display module - 600x480, 7 colors +
software dithering
* `sharp,memory-lcd`: Sharp Memory LCDs - 400x240, 1-bit monochrome
* `solomon-systech,ssd1306`: Solomon Systech SSD1306 - 128x64, 1-bit monochrome
* `sino-wealth,sh1106`: Sino Wealth SH1106 - 128x64, 1-bit monochrome

[SDL Linux display](sdl_display/) is also supported and can be built as an AtomVM plugin.

## Platform Support

AtomGL is currently compatible with **ESP32** microcontrollers. Linux with SDL is also supported for
testing and development purposes.

### Building for ESP32

Place AtomGL in the ESP32 components directory as a regular ESP32 component and build using the
standard ESP-IDF process:

```bash
# Add AtomGL to your ESP32 project's components directory
cd /path/to/AtomVM/src/platforms/esp32/components/
git clone git@github.com:atomvm/atomgl.git

# Build with ESP-IDF
idf.py build
```

### Building for Linux (SDL)

For testing on Linux using SDL, see the [SDL display driver documentation](sdl_display/README.md).

## Getting Started

### Basic Setup

1. Open a display port using the appropriate options
2. Start an [avm_scene](https://github.com/atomvm/avm_scene) to push updates to the display

### Display Configuration Example

Example configuration for ILI9341:

```elixir
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
```

### Direct Port Usage

For direct port communication (without avm_scene):

```erlang
% Create your display list
DisplayList = [
    {text, 10, 20, default16px, 16#000000, transparent, "Hello, World!"},
    {rect, 0, 0, 320, 240, 16#FFFFFF}  % White background
],

% Update the display
port:call(DisplayPort, {update, DisplayList})
```

### Using avm_scene (Recommended)

The `avm_scene` library provides a more ergonomic interface that handles the update mechanics:

```elixir
[...]

def start_link(args, opts) do
  :avm_scene.start_link(__MODULE__, args, opts)
end

[...]

def handle_info(:show_hello, %{width: width, height: height} = state) do
  items = [
    {:text, 10, 20, :default16px, 0x000000, 0x808080, "Hello."},
    {:rect, 0, 0, width, height, 0x808080}
  ]

  {:noreply, state, [{:push, items}]}
end

[...]
```

## Primitives

AtomGL supports the following primitive types:

* **image** - Display bitmap images
* **scaled_cropped_image** - Display scaled and cropped images
* **rect** - Draw filled rectangles
* **text** - Render text with specified font

Example display list with multiple primitives:

```elixir
rendered = [
  {:text, 10, 20, :default16px, 0x000000, 0x808080, "Hello."},
  {:rect, 0, 0, width, height, 0x808080}
]
```

See the [primitives documentation](docs/primitives.md) for detailed information about each primitive type.

## Important Notes

- The display list approach means you always define the complete display state
- Each update replaces the entire previous state (no incremental drawing)
- Z-order is determined by position in the list (first = topmost)
- Always include a background rectangle to ensure proper screen clearing
- The `update` command is the only imperative operation

## License

Apache-2.0
