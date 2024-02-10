<!---
  Copyright 2024 Davide Bettio <davide@uninstall.it>

  SPDX-License-Identifier: Apache-2.0
-->

# Primitives

## image

```erlang
{image,
  X, Y, % image position in pixels, width and height are implicit
  BackgroundColor, % RGB background color, a "hex color" can be used here, or transparent atom
  Image % image tuple
}
```

## scaled_cropped_image

```erlang
{scaled_cropped_image,
  X, Y, Width, Height, % bounding rect in pixels
  BackgroundColor, % RGB background color, a "hex color" can be used here, or transparent atom
  SourceX, SourceY, % offset inside the source image from where the image is taken
  XScaleFactor, YScaleFactor, % integer scaling factor, 1 is original, 2 is twice, etc...
  Opts, % option keyword list, always []: right now no additional options are supported
  Image % image tuple
}
```

## rect

```erlang
{rect,
  X, Y, Width, Height, % bounding rect in pixels
  Color % RGB rectangle color, a "hex color" can be used here
}
```

## text

```erlang
{text,
  X, Y, % text position in pixels, width and height are implicit
  Font, % a font name atom, such as default16px
  TextColor, % RGB background color, a "hex color" can be used here
  BackgroundColor, % RGB background color, a "hex color" can be used here, or transparent atom
  Text % simple text string, UTF-8 can be used, rich text and control characters are not supported
}
```

## Image Tuples

An image tupple contains all the information required for displaying an image.

Such as:
```erlang
{rgba8888, Width, Height, RawPixelBinary}
```
