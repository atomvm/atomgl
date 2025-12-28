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
:erlang.port_call(display, {:update, items}, 5000)
```

**Note:** While direct port calls work, the recommended approach is to use [avm_scene](https://github.com/atomvm/avm_scene) which provides a higher-level interface for managing display updates and handling the display list lifecycle properly.

For more information about display primitives and the display list concept, see the [primitives documentation](primitives.md).
