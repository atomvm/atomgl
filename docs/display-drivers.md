<!--
SPDX-License-Identifier: Apache-2.0
SPDX-FileCopyrightText: AtomGL contributors
-->

# Display Drivers

This document describes how to configure and use the various display drivers supported by AtomGL.

## Overview

To use a display with AtomGL, you need:
1. A communication interface (either SPI or I²C) that must be opened and configured
2. A display driver selected by providing a `compatible` string that matches your display model

The display driver will handle all the low-level communication and rendering, while you provide
display lists describing what should be shown.

## Using SPI Displays

Most displays use SPI for communication. First, configure and open an SPI host, then pass it to the
display driver.

### Basic SPI Setup

```elixir
# Configure SPI bus
spi_opts = %{
  bus_config: %{
    sclk: 35,        # Serial clock pin
    mosi: 34,        # Master Out Slave In pin
    miso: 33,        # Master In Slave Out pin (optional for displays)
    peripheral: "spi2"
  },
  device_config: %{
    # Device-specific configuration
  }
}

# Open SPI host
spi_host = :spi.open(spi_opts)

# Configure display
display_opts = [
  spi_host: spi_host,
  width: 320,
  height: 240,
  compatible: "ilitek,ili9341",
  cs: 22,           # Chip select pin
  dc: 21,           # Data/Command pin
  reset: 18,        # Reset pin
  # Additional options...
]

# Open display port
display = :erlang.open_port({:spawn, "display"}, display_opts)
```

## Using I²C Displays

Some displays (like small OLED screens) use I²C communication.

### Basic I²C Setup

```elixir
# Configure I²C bus
i2c_opts = [
  sda: 8,                    # Data pin
  scl: 9,                    # Clock pin
  clock_speed_hz: 1_000_000,
  peripheral: "i2c0"
]

# Open I²C host
i2c_host = :i2c.open(i2c_opts)

# Configure display
display_opts = [
  i2c_host: i2c_host,
  width: 128,
  height: 64,
  compatible: "solomon-systech,ssd1306",
  invert: true
]

# Open display port
display = :erlang.open_port({:spawn, "display"}, display_opts)
```

## Common Display Options

### Backlight Configuration

Many displays support backlight control:

```elixir
backlight_opts = [
  backlight: 5,              # Backlight GPIO pin
  backlight_active: :low,    # :low or :high
  backlight_enabled: true    # Enable on startup
]
```

## Supported Displays

### ILI9341 / ILI9342C (ilitek,ili9341 / ilitek,ili9342c)

240×320 TFT display with 16-bit colors. Both variants use the same driver.

**Compatible strings:** `"ilitek,ili9341"` or `"ilitek,ili9342c"`

| Option | Type | Description | Default |
|--------|------|-------------|---------|
| `spi_host` | term | SPI host reference | Required |
| `width` | integer | Display width in pixels | 320 |
| `height` | integer | Display height in pixels | 240 |
| `cs` | integer | Chip select GPIO pin | Required |
| `dc` | integer | Data/Command GPIO pin | Required |
| `reset` | integer | Reset GPIO pin | Required |
| `rotation` | integer | Display rotation (0-3) | 0 |
| `enable_tft_invon` | boolean | Enable color inversion | false |
| `backlight` | integer | Backlight GPIO pin | Optional |
| `backlight_active` | atom | Backlight active level (:low/:high) | Optional |
| `backlight_enabled` | boolean | Enable backlight on init | Optional |

**Example:**
```elixir
ili9341_opts = [
  spi_host: spi_host,
  compatible: "ilitek,ili9341",
  width: 320,
  height: 240,
  cs: 22,
  dc: 21,
  reset: 18,
  rotation: 1,
  backlight: 5,
  backlight_active: :low,
  backlight_enabled: true,
  enable_tft_invon: false
]
```

### ILI9486 / ILI9488 (ilitek,ili9486 / ilitek,ili9488)

320×480 TFT displays.

- **ILI9486**: RGB565 over SPI (16-bit color)
- **ILI9488**: RGB666 over SPI (18-bit color; transferred as 3 bytes/pixel). AtomGL renders in RGB565 and converts scanlines for transfer.

**Compatible strings:** `"ilitek,ili9486"` or `"ilitek,ili9488"`

| Option | Type | Description | Default |
|--------|------|-------------|---------|
| `spi_host` | term | SPI host reference | Required |
| `width` | integer | Display width in pixels (kept for API consistency) | 320 |
| `height` | integer | Display height in pixels (kept for API consistency) | 480 |
| `cs` | integer | Chip select GPIO pin | Required |
| `dc` | integer | Data/Command GPIO pin | Required |
| `reset` | integer | Reset GPIO pin | Required |
| `rotation` | integer | Display rotation (0-3) | 0 |
| `enable_tft_invon` | boolean | Enable color inversion | false |
| `color_order` | atom | Color order (:bgr/:rgb) | :bgr |
| `backlight` | integer | Backlight GPIO pin | Optional |
| `backlight_active` | atom | Backlight active level (:low/:high) | Optional |
| `backlight_enabled` | boolean | Enable backlight on init | Optional |

**Example:**
```elixir
ili948x_opts = [
  spi_host: spi_host,
  compatible: "ilitek,ili9488",
  width: 320,
  height: 480,
  cs: 22,
  dc: 21,
  reset: 18,
  rotation: 1,
  enable_tft_invon: false,
  color_order: :bgr
]
```

### ST7789 / ST7796 (sitronix,st7789 / sitronix,st7796)

TFT displays with 16-bit colors.

**Compatible strings:** `"sitronix,st7789"` or `"sitronix,st7796"`

| Option | Type | Description | Default |
|--------|------|-------------|---------|
| `spi_host` | term | SPI host reference | Required |
| `width` | integer | Display width in pixels | 320 |
| `height` | integer | Display height in pixels | 240 |
| `cs` | integer | Chip select GPIO pin | Required |
| `dc` | integer | Data/Command GPIO pin | Required |
| `reset` | integer | Reset GPIO pin | Optional |
| `rotation` | integer | Display rotation (0-3) | 0 |
| `x_offset` | integer | X-axis offset in pixels | 0 |
| `y_offset` | integer | Y-axis offset in pixels | 0 |
| `enable_tft_invon` | boolean | Enable color inversion | false |
| `init_list` | list | Custom initialization sequence | Optional |
| `backlight` | integer | Backlight GPIO pin | Optional |
| `backlight_active` | atom | Backlight active level (:low/:high) | Optional |
| `backlight_enabled` | boolean | Enable backlight on init | Optional |

**Example with custom initialization:**
```elixir
st7796_opts = [
  spi_host: spi_host,
  compatible: "sitronix,st7796",
  width: 480,
  height: 222,
  y_offset: 49,
  cs: 38,
  dc: 37,
  init_list: [
    {0x01, <<0x00>>}, # {command, <<data>>}
    {:sleep_ms, 120}  # wait 120 ms
		# ...
  ]
]
```

### SSD1306 / SH1106 (solomon-systech,ssd1306 / sino-wealth,sh1106)

128×64 monochrome OLED displays using I²C communication.

**Compatible strings:** `"solomon-systech,ssd1306"` or `"sino-wealth,sh1106"`

| Option | Type | Description | Default |
|--------|------|-------------|---------|
| `i2c_host` | term | I²C host reference | Required |
| `width` | integer | Display width in pixels | 128 |
| `height` | integer | Display height in pixels | 64 |
| `reset` | integer | Reset GPIO pin | Optional |
| `invert` | boolean | Invert display colors | false |

**Example:**
```elixir
ssd1306_opts = [
  i2c_host: i2c_host,
  compatible: "solomon-systech,ssd1306",
  width: 128,
  height: 64,
  invert: false,
  reset: 16  # Optional
]
```

### Sharp Memory LCD (sharp,memory-lcd)

400×240 monochrome memory LCD with ultra-low power consumption.

**Compatible string:** `"sharp,memory-lcd"`

| Option | Type | Description | Default |
|--------|------|-------------|---------|
| `spi_host` | term | SPI host reference | Required |
| `width` | integer | Display width in pixels | 400 |
| `height` | integer | Display height in pixels | 240 |
| `cs` | integer | Chip select GPIO pin | Required |
| `en` | integer | Enable GPIO pin | Optional |

**Example:**
```elixir
sharp_lcd_opts = [
  spi_host: spi_host,
  compatible: "sharp,memory-lcd",
  cs: 22,
  en: 23  # Optional enable pin
]
```

### Waveshare 5.65" ACeP 7-Color (waveshare,5in65-acep-7c)

600×480 7-color E-Paper display. Driver has software dithering support.

**Compatible string:** `"waveshare,5in65-acep-7c"`

| Option | Type | Description | Default |
|--------|------|-------------|---------|
| `spi_host` | term | SPI host reference | Required |
| `width` | integer | Display width in pixels | 600 |
| `height` | integer | Display height in pixels | 480 |
| `cs` | integer | Chip select GPIO pin | Required |
| `dc` | integer | Data/Command GPIO pin | Required |
| `reset` | integer | Reset GPIO pin | Required |
| `busy` | integer | Busy signal GPIO pin | Required |

**Note:** E-Paper displays have slow refresh rates due to their technology.

**Example:**
```elixir
epaper_opts = [
  spi_host: spi_host,
  compatible: "waveshare,5in65-acep-7c",
  cs: 22,
  dc: 21,
  reset: 18,
  busy: 19
]
```

### E-Paper Display API

AtomGL provides a data-driven descriptor framework to drive monochrome and 4-gray E-Paper displays. Compatible panel configurations are represented as bytecode descriptors, meaning the same driver binary can adapt to different controllers, native geometry vs. user-facing orientation, byte/bit layouts, and waveform program payloads.

#### Panel Constructors

To obtain a panel descriptor, call `atomgl_epaper:panel/1` or `atomgl_epaper:panel/2`.

```elixir
# Default panel configuration (landscape)
{:ok, descriptor} = :atomgl_epaper.panel("waveshare,epd2in9_V2")

# Custom panel configuration with options
{:ok, descriptor_portrait} = :atomgl_epaper.panel("waveshare,epd2in9_V2", %{
  orientation: :portrait,
  default_refresh: :partial
})
```

Supported compatibility lookup strings for panels:
- **SSD1680 2.9"**: `"waveshare,epaper-2in9"`, `"waveshare,epd2in9_V2"`, `"waveshare,epd2in9_V2-fast"`, `"waveshare,epd2in9_V2-partial"`, `"waveshare,epd2in9_V2-4gray"`, `"dke,depg0290bns800f6"`
- **SSD1680 2.13"**: `"waveshare,epaper-2in13"`, `"waveshare,epd2in13_V4"`, `"waveshare,epd2in13_V4-fast"`, `"waveshare,epd2in13_V4-partial"`
- **JD79656 2.13"**: `"heltec,lcmen2r13efc1"`, `"heltec,icmen2r13efc1"`, `"heltec,ht-vme213"`

##### Constructor Options

When calling `panel/2`, you can customize:
- `orientation` - `:landscape`, `:landscape_left`, `:landscape_right`, `:portrait`, or `:portrait_flipped` (automatically maps native coordinates to view width/height and sets rotation). `:landscape` is a compatibility alias for `:landscape_left`.
- `default_refresh` - The refresh mode used when no frame option is specified.
- `refresh_modes` - List of allowed refresh modes (e.g., `[:full, :fast, :partial]`).
- `ghosting` - Map with ghosting settings:
  - `max_fast_refreshes` - Maximum successive fast updates before a full refresh is forced.
  - `reseed_on_timeout` - Reseed previous/current planes with full refresh after BUSY timeouts.
- `timing` - Map to adjust driver timeout parameters (`full_expected_ms`, `fast_expected_ms`, `poll_interval_ms`, `timeout_ms`).

#### E-Paper Descriptor Schema

A completed descriptor is a keyword list with the following shape:

```elixir
[
  descriptor_version: 2,
  name: "Waveshare epd2in9_V2 2.9\" e-paper SSD1680 v1",
  controller: :ssd16xx, # :ssd16xx, :jd79656, :uc8175, or :acep7
  native_width: 128,
  native_height: 296,
  view_width: 296,
  view_height: 128,
  rotation: 90,
  spi_clock_hz: 4000000,
  busy_idle_level: 0,
  use_gpio_pullups: false,
  frame_layout: :row_msb, # :row_msb, :row_lsb, :column_msb, :column_lsb
  polarity: :white_1,    # :white_1 or :black_1
  refresh_modes: [:full, :fast, :partial, :4gray],
  default_refresh: :full,
  programs: %{
    init: <<...>>,    # Init bytecode
    full: <<...>>,    # Full refresh waveform/cmd sequence
    fast: <<...>>,    # Fast refresh sequence (optional)
    partial: <<...>>, # Partial refresh sequence (optional)
    sleep: <<...>>,   # Sleep command sequence (optional)
    "4gray": <<...>>  # 4-gray refresh sequence (optional)
  },
  timing: %{
    full_expected_ms: 2000,
    fast_expected_ms: 500,
    poll_interval_ms: 50,
    timeout_ms: 5000
  },
  ghosting: %{
    max_fast_refreshes: 10,
    reseed_on_timeout: true
  },
  lut_full: <<...>>,    # Waveform LUT (optional)
  lut_partial: <<...>>, # Waveform LUT (optional)
  lut_4gray: <<...>>,   # Waveform LUT (optional)
  lut_fast: <<...>>     # Waveform LUT (optional)
]
```

#### Ghosting Policy

E-Paper controllers are highly susceptible to image burn-in and visual ghosting. AtomGL implements a conservative driver-level ghosting policy:
1. **Reseeding**: When the previous frame state is unknown or invalid, a partial/fast update is promoted to a full refresh to seed the controller's current/previous memory.
2. **Promotion Count**: After `max_fast_refreshes` successive fast/partial updates, the next update is automatically promoted to a full refresh to clean the screen.
3. **BUSY Timeout**: If the controller fails to finish in time (timing out on wait), the driver invalidates the previous frame and schedules a full refresh on the next update.

#### Opening the Port

Configure the SPI display port with the descriptor returned by the constructor:

```elixir
epaper_opts = [
  spi_host: spi_host,
  compatible: "waveshare,epd2in9_V2",
  descriptor: descriptor,
  cs: 22,
  dc: 21,
  reset: 18,
  busy: 19
]

display = :erlang.open_port({:spawn, "display"}, epaper_opts)
```

#### Runtime Refresh Options

When sending updates via the port, you can dynamically choose the refresh mode per frame (rather than being locked to one mode at port creation time):

```elixir
# Standard/Default update
:port.call(display, {:update, items}, 5000)

# Request a fast refresh mode for this frame
:port.call(display, {:update, items, [refresh: :fast]}, 5000)
```

If the requested refresh mode is not listed in the descriptor's `refresh_modes`, the driver defaults back to `default_refresh`. Only modes defined and present in the descriptor are permitted.

## Custom Initialization Sequences

Many displays require specific initialization sequences with carefully tuned values for voltages,
timing, gamma curves, and other display-specific parameters. For displays that need custom
initialization, you can provide an `init_list`:

```elixir
init_list: [
  {0x01, <<0x00>>},
  {:sleep_ms, 120}
	# ...
]
```

Each entry can be:
- `{command, data}` - Send command byte followed by data bytes
- `{:sleep_ms, milliseconds}` - Delay for specified time

These sequences are highly specific to each display model and typically come from the manufacturer's datasheet or reference implementation.

## Updating the Display

Once configured, update the display using the display port:

```elixir
# Create display list
items = [
  {:text, 10, 20, :default16px, 0x000000, 0xFFFFFF, "Hello, World!"},
  {:rect, 0, 0, 320, 240, 0xFFFFFF}  # White background
]

# Send update command
:port.call(display, {:update, items}, 5000)
```

**Note:** While direct port calls work, the recommended approach is to use [avm_scene](https://github.com/atomvm/avm_scene) which provides a higher-level interface for managing display updates and handling the display list lifecycle properly.

For more information about display primitives and the display list concept, see the [primitives documentation](primitives.md).
