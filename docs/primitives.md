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
displays may reduce color depth as needed. A color can be any integer, but only its low 24 bits are
used: `16#1FF0000` is the same red and `-1` is white.

### Coordinates and Sizes
All numeric values are integers, of any size AtomVM can represent up to 64 bits. Coordinates are
specified in pixels, as are sizes. Subpixel or half-pixel values are not allowed.

- `rect` and `text` accept any coordinate and size. Values beyond ±32767 are clamped to that range
  without changing which pixels are drawn on screen, so `{rect, 0, 0, 100000, 100000, Color}`
  still fills the screen. A `rect` with a width or height of 0 or less draws nothing.
- `image` and `scaled_cropped_image` coordinates, sizes, source offsets and scale factors, image
  widths and heights, and every value of a shape (coordinates, sizes and radii) must be within
  ±32767, or the item is invalid.

### Invalid Items
An item with a wrong arity, a value of the wrong type or out of range, or an unknown command is
skipped: it draws nothing and the rest of the display list is drawn as usual. Each skipped item is
logged to stderr, one line per item:

```
invalid display list item N (Command/Arity): Reason
```

`N` is the item's position in the display list, starting at 1. `(Command/Arity)` is left out when
the item is not a tuple, and `Command` is `tuple` when the first element is not an atom. For
example:

```
invalid display list item 2 (rect/5): wrong arity
invalid display list item 3: not a command tuple
invalid display list item 4 (tuple/2): unknown command
```

At most 3 invalid items are logged per update. If there are more, one more line gives their number,
such as `2 more invalid display list items`.

An empty display list draws only the background. A display list that is not a proper list is
rejected with `invalid display list: not a proper list` and the update is skipped.

### Transparent
The `transparent` atom indicates that no background is drawn for the item's bounding rectangle,
allowing the item to be properly rendered over lower items in the display list. This may have
performance implications.

### Text
Text can be provided as either an Erlang string (a list) or an Elixir string (a binary). UTF-8
encoding is supported.

### Shapes
Shape primitives (`rounded_rect`, `circle`, `ellipse`) paint only the pixels inside the shape;
pixels in the bounding box but outside the shape show whatever item is below in the display list.

`circle` and `ellipse` place their points on pixel centers: the pixel at `{X, Y}` is
drawn when it is inside. Round edges use the midpoint rule: a pixel at offset `{DX, DY}` from the
center is inside a radius `R` when `DX * DX + DY * DY < R * R + R`, so a shape of radius `R` is
`2 * R + 1` pixels across and a circle of radius 1 is a 5 pixel plus. An ellipse applies the same
rule to each axis.

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
The item is invalid unless `0 <= SourceX < ImageWidth`, `0 <= SourceY < ImageHeight`, the scale
factors are at least 1 and `Width` and `Height` are at least 0. `Width` and `Height` are reduced to
what is left of the image right of and below the source offset, times the scale factor.

`Opts` is a list. `flip_x` or `{flip_x, true}` mirrors the item horizontally, `flip_y` or
`{flip_y, true}` vertically. Other entries are ignored, and so is an `Opts` that is not a list.
A flip mirrors the drawn pixels in place, not the whole source image: with `flip_x`, the pixel at
offset `C` from the item's left edge shows what the unflipped item shows at offset `Width - 1 - C`,
also when `Width` is not a multiple of the scale factor.

```erlang
{scaled_cropped_image,
  X, Y, Width, Height, % bounding rect in pixels
  BackgroundColor, % RGB background color, a "hex color" can be used here, or transparent atom
  SourceX, SourceY, % offset inside the source image from where the image is taken
  XScaleFactor, YScaleFactor, % integer scaling factor, 1 is original, 2 is twice, etc.
  Opts, % option list: [flip_x], [flip_y], [flip_x, flip_y], or [] for none
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

Renders text with the specified font and colors. `Font` must be an atom, or the item is invalid.
With the built-in font the text is 8 pixels wide per character and 16 pixels high. On builds
without ufont support, a font other than `default16px` falls back to the built-in font and logs
`unsupported font: Font` on every update. With ufont support, a font that was not registered makes
the item invalid.

```erlang
{text,
  X, Y, % text position in pixels, width and height are implicit
  Font, % a font name atom, such as default16px
  TextColor, % RGB text color, a "hex color" can be used here
  BackgroundColor, % RGB background color, a "hex color" can be used here, or transparent atom
  Text % simple text string, UTF-8 can be used, rich text and control characters are not supported
}
```

## rounded_rect

Draws a filled rectangle with rounded corners. The corners are quarter circles centered on the
pixels `Radius` pixels in from each corner, so a `2 * R + 1` square with radius `R` is the same as a
circle of radius `R`. The radius clamp keeps a straight edge of at least one pixel on every side:
a 12 pixel high button gets a radius of at most 5.

```erlang
{rounded_rect,
  X, Y, Width, Height, % bounding rect in pixels
  Radius, % corner radius in pixels, >= 0, clamped to (min(Width, Height) - 1) div 2
  Color % RGB fill color
}
```

## circle

Draws a filled circle centered on the given point.

```erlang
{circle,
  CX, CY, % center in pixels
  R, % radius in pixels, > 0, the circle is 2 * R + 1 pixels wide
  Color % RGB fill color
}
```

## ellipse

Draws a filled ellipse centered on the given point.

```erlang
{ellipse,
  CX, CY, % center in pixels
  RX, RY, % horizontal and vertical radius in pixels, > 0
  Color % RGB fill color
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

Only `rgba8888` is supported. Width and height must be between 1 and 32767 and the binary must hold
at least `Width * Height * 4` bytes, otherwise the item is invalid. Width and height must match the
dimensions of the image data in the binary; wrong values that pass this check draw a corrupted
image.

**Tip:** You can convert images to raw RGBA format using ImageMagick:
```bash
convert -define h:format=rgba -depth 8 image.png image.rgba
```
