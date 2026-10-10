<!--
This file is part of AtomGL.

Copyright 2026 Tom Hoenderdos <tomhoenderdos@gmail.com>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

SPDX-License-Identifier: Apache-2.0
-->

# Renderer benchmark and sprite tests

This directory builds two host programs against the real renderer sources.
They need AtomVM's headers (`display_items.h` includes `context.h`), which
is why they live here and not in `tests/shapes`:

- `bench`: times the DCS LCD scanline renderer over a set of scenes.
- `test_flip`: checks that `scaled_cropped_image` source pixel lookups,
  flipped or not, stay inside the source image and return the same
  pixels as a reference mapping kept in the test.

Build and run (`LIBATOMVM_INCLUDE_PATH` points at AtomVM's
`src/libAtomVM`; only headers are used, nothing from AtomVM is linked):

```sh
cmake -S tests/bench -B build/bench -DLIBATOMVM_INCLUDE_PATH=/path/to/AtomVM/src/libAtomVM
cmake --build build/bench
./build/bench/bench
./build/bench/test_flip
```

## Host benchmark

`bench.c` calls `dcs_lcd_draw_x()`, the function the ESP32 LCD driver
calls for every pixel run, over a 240x240 screen with a single line
buffer, and builds each row's item list with `display_items_row()` as
the driver does. No ESP-IDF code is linked; `tests/stubs/driver/spi_master.h`
stands in for the one ESP-IDF header the renderer includes.

Each scene is drawn over a full-screen background rect. The first table
gives the mean time per frame (240 lines, after a warm-up) and per line.
The second table repeats every scene that contains shapes with each shape
replaced by a `rect` of its bounding box: the ratio is what the shapes
cost over flat rects of the same size.

The scenes cover single shapes, grids of small shapes, 50 overlapping
circles, 12 lines and 5 arcs that all cross the same rows, a 64-point
comb polygon (the most points a polygon may have, with about 32 edge
crossings on every row) next to a 4-point polygon of the same bounding
box, a text UI with 25 text items, and full-screen sprites at scale 1
and 3, plain and flipped.

Scaled images are drawn one source pixel at a time, which suits the
ESP32 build (optimized for size with -Os) but does not vectorize on
the host: the sprite scenes at scale 1 are much slower here than on a
build that draws pixel by pixel, while on the device they are not.

Host numbers are only meaningful relative to each other. The host has no
SPI bus and its CPU is much faster than the ESP32-S3's, so they do not
predict frame rates on the device. Example output (Apple Silicon, -O3):

```
scene                        us/frame   us/line
background only                   4.3      0.02
rounded_rect                      8.0      0.03
circle                            9.7      0.04
ellipse                           9.4      0.04
thin line                         4.7      0.02
diagonal thick line              12.6      0.05
arc gauge                        18.8      0.08
ring                             16.0      0.07
full-screen circle               18.8      0.08
star polygon                     14.0      0.06
20 bullets                       27.3      0.11
50 overlapping bullets          126.6      0.53
6 buttons                        20.0      0.08
12 crossing lines                77.0      0.32
5 concentric arcs               122.1      0.51
64-point comb polygon            76.5      0.32
4-point polygon, comb bbox        6.7      0.03
text UI (25 text items)          74.6      0.31
sprite x1                       224.5      0.94
sprite x3                        78.9      0.33
sprite x1 flip x                232.7      0.97
sprite x1 flip xy               230.9      0.96
sprite x3 flip x                 79.1      0.33
sprite x3 flip xy                79.4      0.33

shapes vs. rects of their bounding boxes
scene                        shape us    rect us   ratio
rounded_rect                      8.0        4.5    1.78
circle                            9.7        4.5    2.17
ellipse                           9.4        4.5    2.08
thin line                         4.7        4.3    1.09
diagonal thick line              12.6        5.5    2.27
arc gauge                        18.8        5.1    3.70
ring                             16.0        5.1    3.11
full-screen circle               18.8        4.3    4.34
star polygon                     14.0        5.3    2.66
20 bullets                       27.3       13.7    2.00
50 overlapping bullets          126.6       36.1    3.50
6 buttons                        20.0        8.7    2.29
12 crossing lines                77.0        4.4   17.56
5 concentric arcs               122.1       13.4    9.09
64-point comb polygon            76.5        5.5   14.01
4-point polygon, comb bbox        6.7        5.3    1.26

64-point comb over a 4-point polygon of its bbox: 11.43x
```

The rect version of a scene of long lines is a stack of full-screen
rects, of which only the top one is drawn, hence the large ratios for
the crossing lines and the comb.

## Line budget on the ESP32-S3

The driver draws each line while the previous one is sent over SPI DMA
(see below), so what matters on the device is how long a line takes to
draw compared to how long it takes to send. The time to send a line
depends on the panel and the SPI clock:

```
line_time = width * bits_per_pixel / spi_clock
```

For example:

- 240 px RGB565 at 40 MHz: 240 * 16 / 40 MHz = 96 us.
- 320 px RGB565 at 80 MHz: 320 * 16 / 80 MHz = 64 us.

A line that takes longer than `line_time` to draw delays the next
transfer.

The table estimates the slowest line of each scene on an ESP32-S3 at
160 MHz. The estimates come from counting, per line, the calls and loop
steps of the renderer and the shape code on the host, and weighting each
with a low and a high cycle count for the Xtensa LX7 running from flash
cache. They are estimates, not measurements; `ATOMGL_PROFILE` gives the
real numbers.

```
scene                        slowest line, us
background only                   2 - 3
rounded_rect                      7 - 13
circle                            7 - 13
ellipse                           8 - 14
thin line                         6 - 10
diagonal thick line               6 - 10
arc gauge                        16 - 30
ring                             12 - 22
full-screen circle                7 - 13
star polygon                      7 - 13
20 bullets                       41 - 74
6 buttons                        14 - 26
12 crossing lines                50 - 92
sprite x1, x3, flipped or not    18 - 30
5 concentric arcs                77 - 148
50 overlapping bullets          143 - 254
```

A line costs roughly a fixed amount per item that crosses it plus a
fixed amount per run of pixels the line breaks into, so scenes stay
within the line time as long as each line stays modest (the figures
below are for the 96 us of a 240 px panel at 40 MHz):

- Long lines: about a dozen crossing the same line, at 4 - 8 us each.
- Arcs: 15 - 30 us each where they cross a line; about 3 to 6 on the
  same line. Each arc solves its row from scratch.
- Polygons: about 1 us per edge crossing on a line, so up to roughly 60
  crossings per line. A convex polygon has 2, whatever its number of
  points. A line where many edges start costs more, since each of them
  is set up there.
- Many overlapping items: every draw call walks the items that cover
  the line, so a line where 50 items overlap and break it into 40 runs
  costs 2000 item visits. Items above or below the line cost only one
  bounding box test per line. Transparent text and image pixels are
  skipped as whole runs, so text without a background costs about one
  run per glyph stroke.

Past the line time the frame only gets longer: each slow line delays
the next transfer by its excess.

## On-device profiling

`dcs_lcd_display_driver.c` can time each display update. The timer is
compiled in only when the firmware is built with `ATOMGL_PROFILE`, and
costs nothing otherwise:

```sh
idf.py -DATOMGL_PROFILE=1 build flash monitor
```

Each update then logs one line to stderr (visible in `idf.py monitor`):

```
atomgl: <n> items, parse <parse_us> us, draw <draw_us> us, max line <max_line_us> us, spi wait <spi_wait_us> us, frame <frame_us> us
```

- `n` is the number of items in the display list.
- `parse` is the time spent turning the display list into items
  (`display_items_init_list()`).
- `draw` is the sum over all lines of the time spent drawing them with
  `dcs_lcd_draw_x()`. It does not include SPI waits.
- `max line` is the drawing time of the slowest line, without SPI
  waits.
- `spi wait` is the sum of the time spent blocked in
  `spi_device_get_trans_result()`, waiting for the previous line's
  transfer to finish.
- `frame` is the whole update: parsing, drawing, SPI waits, queueing
  the transfers, and freeing the items.

All times come from `esp_timer_get_time()`, so they are wall-clock
times: if the display task is preempted by another task or an
interrupt while drawing a line, that time is counted too. A single
large `max line` can be preemption; look at many frames, and at `draw`
divided by the number of lines, before blaming one line.

The driver draws each line while the previous one is sent over SPI
DMA, so each line costs about the larger of its drawing time and
`line_time` (see above). When `draw` divided by the number of lines is
well below `line_time`, most of the frame is `spi wait` and drawing is
hidden behind the transfers. When `spi wait` is close to zero, drawing
is the bottleneck and the frame gets longer with every slow line.

To try a shape on the device, send it a scene with `port:call/2`, as in
the top-level README:

```erlang
Scene = [
    {circle, 120, 120, 50, 16#20C040},
    {rect, 0, 0, 240, 240, 16#202020}
],
ok = port:call(Display, {update, Scene})
```

Replace the circle with any other item from `docs/primitives.md` and
compare `max line` in the monitor output.
