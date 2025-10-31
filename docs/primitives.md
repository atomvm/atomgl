<!---
  Copyright 2024 Davide Bettio <davide@uninstall.it>

  SPDX-License-Identifier: Apache-2.0
-->

# Primitives

AtomGL primitives are the basic drawing elements that make up a display list. Each primitive is
represented as an Erlang tuple with specific parameters defining its appearance and position.

## Types

### Colors
Colors are represented as 24-bit RGB values. For example, `0xFF0000` represents red (equivalent to
HTML color `#FF0000`). Display drivers for monochrome devices may apply dithering, while 16-bit
displays may reduce color depth as needed.

### Coordinates and Sizes
All numeric values are integers. Coordinates are specified in pixels, as are sizes. Subpixel or
half-pixel values are not allowed.

### Transparent
The `transparent` atom indicates that no background is drawn for the item's bounding rectangle,
allowing the item to be properly rendered over lower items in the display list. This may have
performance implications.

### Text
Text can be provided as either an Erlang string (a list) or an Elixir string (a binary). UTF-8
encoding is supported.

## image

Displays an image at the specified position. The image dimensions are determined by the image tuple
itself.

```erlang
{image,
  X, Y, % image position in pixels, width and height are implicit
  BackgroundColor, % RGB background color, a "hex color" can be used here, or transparent atom
  Image % image tuple
}
```

## scaled_cropped_image

Displays a portion of an image with scaling applied. Useful for sprite sheets or zoomed views.

```erlang
{scaled_cropped_image,
  X, Y, Width, Height, % bounding rect in pixels
  BackgroundColor, % RGB background color, a "hex color" can be used here, or transparent atom
  SourceX, SourceY, % offset inside the source image from where the image is taken
  XScaleFactor, YScaleFactor, % integer scaling factor, 1 is original, 2 is twice, etc.
  Opts, % option keyword list, always []: right now no additional options are supported
  Image % image tuple
}
```

## rect

Draws a filled rectangle with the specified color.

```erlang
{rect,
  X, Y, Width, Height, % bounding rect in pixels
  Color % RGB rectangle color, a "hex color" can be used here
}
```

## text

Renders text with the specified font and colors.

```erlang
{text,
  X, Y, % text position in pixels, width and height are implicit
  Font, % a font name atom, such as default16px
  TextColor, % RGB text color, a "hex color" can be used here
  BackgroundColor, % RGB background color, a "hex color" can be used here, or transparent atom
  Text % simple text string, UTF-8 can be used, rich text and control characters are not supported
}
```

## Image Tuples

An image tuple contains all the information required for displaying an image. Images are tagged
tuples with the following structure:

```erlang
{Format, Width, Height, RawPixelBinary}
```

The format tag indicates the pixel format. For example, `rgba8888` means:
- RGBA format with alpha channel
- Byte order: R, G, B, A
- Each component is 8 bits

**Important:** Width and height must exactly match the dimensions of the image data in the binary.
Incorrect values will result in corrupted image display.

**Tip:** You can convert images to raw RGBA format using ImageMagick:
```bash
convert -define h:format=rgba -depth 8 image.png image.rgba
```
