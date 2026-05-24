<!-- SPDX-License-Identifier: Apache-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com> -->

# Adding E-Paper Panels

This guide is for humans and LLM agents adding a new E-Paper panel descriptor to
AtomGL. The descriptor library now lives in this `atomgl_epaper` package; the
low-level C display driver still lives in the parent AtomGL component. Keep
panel additions small, traceable, and license-clean.

## Source Policy

Prefer primary, permissively licensed sources:

- Vendor datasheets, product specs, and command tables.
- Vendor driver examples with compatible licenses, such as
  [waveshareteam/e-Paper](https://github.com/waveshareteam/e-Paper).
- Existing descriptors in `atomgl_epaper/src/atomgl_epaper_panels.erl` and driver
  helpers in the parent AtomGL tree.

The Waveshare repository is a useful starting point for many panel init
sequences, refresh commands, LUT payloads, reset timing, BUSY polarity, and
native dimensions. When copying bytes or command sequences from permissively
licensed source, preserve the relevant attribution or license notice near the
copied data.

Treat GPL code as reference only:

- [bitbank2/bb_epaper](https://github.com/bitbank2/bb_epaper) and similar GPL
  projects are useful for identifying controller families, panel aliases, and
  behavioral clues.
- Do not copy GPL code, LUT tables, command byte arrays, comments, names chosen
  only by that project, or implementation structure into AtomGL.
- If GPL code helped orient the work, describe it as corroborating reference
  material in the commit or PR notes, not as the source for copied content.

If the license of a source is unclear, do not copy from it. Use it only to form
questions, then verify details from a primary source or hardware testing.

## Before Editing

Collect these facts for the target panel:

- Panel marketing name and likely compatible lookup strings.
- Native width and height, not just the desired display orientation.
- Controller IC or controller family. If uncertain, name the family rather than
  inventing a precise controller.
- BUSY idle level, reset pulse timing, and whether GPIO pullups are needed.
- RAM data commands for old/current planes or color planes.
- Frame layout: row-major or column-major, MSB or LSB first.
- Pixel polarity: whether `1` means white or black.
- Refresh modes: full, fast, partial, 4-gray, color, sleep.
- LUT format and whether extra register bytes accompany the LUT payload.
- Any controller-specific plane mapping, palette, or ghosting constraints.

Use `rg` to search this package, the parent driver, and source trees for similar
panels first. Match the existing descriptor style instead of inventing a new
shape.

Important local paths:

- `atomgl_epaper/src/atomgl_epaper.erl` - public `panel/1`, `panel/2`, and
  descriptor validation facade.
- `atomgl_epaper/src/atomgl_epaper_panels.erl` - panel lookup strings,
  controller command sequences, and LUTs.
- `atomgl_epaper/src/atomgl_epaper_descriptor.erl` - descriptor builders,
  orientation handling, timing defaults, and validation.
- `atomgl_epaper/src/atomgl_epaper_program.erl` - bytecode helpers for
  commands, delays, BUSY waits, plane insertion, and LUT insertion.
- `atomgl_epaper/test/atomgl_epaper_tests.erl` - descriptor and validation
  tests.
- `atomgl_epaper/README.md` - package usage and public package notes.
- `atomgl_epaper/CUSTOM_PANEL.md` - userland descriptor prototyping guide.
- `epaper_commands.h` and `epaper_display_driver.c` - parent AtomGL C driver
  files, needed only when controller behavior changes.

## Implementation Checklist

1. Decide whether this is descriptor-only

   Most panels should be descriptor-only changes in
   `atomgl_epaper/src/atomgl_epaper_panels.erl`. Reuse an existing controller
   when the command set and driver behavior match.

   Add a new controller only when parsing, LUT insertion, plane writes, palette
   mapping, refresh flow, or BUSY behavior needs distinct C-side handling.

   Controller changes usually touch:

   - `epaper_commands.h`
   - `epaper_display_driver.c`

2. Add a panel descriptor

   Panel lookup clauses live in `atomgl_epaper/src/atomgl_epaper_panels.erl`.
   Add:

   - A primary lookup string, usually vendor-qualified.
   - Optional aliases for mode defaults, such as `-fast`, `-partial`, or
     `-4gray`.
   - Native geometry, palette size, refresh modes, and default refresh mode.
   - Init, refresh, partial, 4-gray, LUT, and sleep programs as needed.
   - Helper functions only when they reduce duplication or make the sequence
     easier to audit.

3. Keep driver behavior generic

   The C driver should know about controller behavior, not product names. Avoid
   hardcoding a specific panel lookup string in the driver. Product-specific
   command values should usually live in the descriptor bytecode.

4. Document the lookup strings

   Update package docs when a new public lookup string is added. If the parent
   AtomGL docs include a supported-panel list, update that too.

5. Add descriptor tests

   Add focused EUnit coverage in `atomgl_epaper/test/atomgl_epaper_tests.erl`
   for:

   - Controller atom.
   - Native and view dimensions.
   - Refresh mode list and default mode aliases.
   - LUT byte size or palette size when relevant.

## Verification

Run the lightweight descriptor checks:

```sh
cd atomgl_epaper
rebar3 eunit
rebar3 xref
cd ..
git diff --check
```

For descriptor-only changes, `rebar3 eunit` is the main package test. For
controller or C driver changes, also run the broader parent AtomGL tests
available in your environment. If hardware is available, verify:

- Full refresh clears and draws correctly.
- Fast or partial refresh updates the expected window without stale-plane
  artifacts.
- Sleep and wake work repeatedly.
- 4-gray or color palettes map white, gray/color levels, and black correctly.
- Timeout behavior matches the panel BUSY polarity.

Record which hardware revision was tested. Some Waveshare panels changed
controller behavior across V1/V2/V3 or after a production date.

## LLM Agent Rules

When an LLM agent adds a panel:

- State which source files or datasheets were used and their licenses.
- Use GPL sources only as reference; never copy bytes, arrays, or comments from
  them.
- Prefer Waveshare or other permissive vendor sources for command bytes and LUTs
  when available.
- Do not claim an exact controller when the evidence only says `UC81xx`,
  `SSD16xx`, or another family name.
- Keep the change in a separate commit for each controller or panel family.
- Keep descriptor tests in `atomgl_epaper/test`, not inline in the descriptor
  module.
- Do not refactor unrelated display code while adding a panel.
- Run the verification commands above and report failures honestly.

When in doubt, leave a narrowly worded note in the descriptor name or docs, such
as `UC81xx-compatible`, instead of encoding a guessed controller identity.
