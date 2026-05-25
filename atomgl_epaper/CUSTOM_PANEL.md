<!-- SPDX-License-Identifier: Apache-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com> -->

# Custom E-Paper Panels

You can prototype a panel descriptor in your own Erlang or Elixir project
without adding a lookup string to AtomGL. Use `atomgl_epaper_descriptor` to
build and validate the descriptor, and use `atomgl_epaper_program` to build the
bytecode programs consumed by the C display driver.

Once the descriptor works on hardware, it can be copied into
`atomgl_epaper/src/atomgl_epaper_panels.erl` with a public lookup string and a
focused descriptor test.

## Example: Waveshare 1.54 V2 200x200

The example below is a userland descriptor module for the Waveshare 1.54 inch
V2 200x200 monochrome panel. The command sequence and waveform bytes are
adapted from Waveshare's MIT-licensed `EPD_1in54_V2` driver:

- `Arduino_R4/src/e-Paper/EPD_1in54_V2.cpp`
- `Arduino_R4/src/e-Paper/EPD_1in54_V2.h`

The AtomGL controller atom is `ssd16xx`; this covers SSD1680-compatible
program-driven monochrome panels.

```erlang
%% SPDX-License-Identifier: MIT
%% SPDX-SnippetCopyrightText: Waveshare team

-module(my_epaper_200x200).

-export([
    descriptor/0,
    descriptor/1
]).

-import(atomgl_epaper_program, [
    program/1,
    cmd/1,
    cmd/2,
    wait_busy/2,
    reset/3,
    insert_plane/1,
    insert_lut/1,
    insert_prev_frame/1,
    capture_frame/0,
    mark_prev_valid/0,
    delay_ms/1
]).

-define(LUT_FULL, 0).
-define(LUT_PARTIAL, 1).

descriptor() ->
    descriptor(#{}).

descriptor(Opts) ->
    atomgl_epaper_descriptor:ssd16xx_panel(
        "Waveshare 1.54 V2 200x200 e-paper SSD16xx-compatible",
        maps:merge(#{
            native_width => 200,
            native_height => 200,
            refresh_modes => [full, partial],
            default_refresh => full,
            init => init(),
            full => full_refresh(),
            partial => partial_refresh(),
            sleep_modes => [
                {sleep, [
                    {enter, cmd(16#10, <<16#01>>)},
                    {wake, reset_init},
                    {controller_ram, unknown},
                    {host_prev_frame, preserve},
                    {after_wake_refresh, allow_if_program_reseeds}
                ]}
            ],
            lut_full => wf_full_1in54(),
            lut_partial => wf_partial_1in54()
        }, Opts)).

%% `allow_if_program_reseeds` allows fast/partial after wake only when the
%% selected refresh program includes insert_prev_frame/1.

init() ->
    program([
        reset(200, 2, 200),
        wait_busy(0, 5000),
        cmd(16#12),
        wait_busy(0, 5000),
        cmd(16#01, <<16#C7, 16#00, 16#01>>),
        cmd(16#11, <<16#01>>),
        full_window(),
        cmd(16#3C, <<16#01>>),
        cmd(16#18, <<16#80>>),
        cmd(16#22, <<16#B1>>),
        cmd(16#20),
        set_ram_cursor(),
        wait_busy(0, 5000),
        insert_lut(?LUT_FULL)
    ]).

full_refresh() ->
    program([
        insert_lut(?LUT_FULL),
        set_ram_cursor(),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        delay_ms(2),
        set_ram_cursor(),
        cmd(16#26),
        insert_prev_frame(0),
        cmd(16#22, <<16#C7>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

partial_refresh() ->
    program([
        insert_lut(?LUT_PARTIAL),
        cmd(16#37, <<16#00, 16#00, 16#00, 16#00, 16#00,
                    16#40, 16#00, 16#00, 16#00, 16#00>>),
        cmd(16#3C, <<16#80>>),
        cmd(16#22, <<16#C0>>),
        cmd(16#20),
        wait_busy(0, 5000),
        set_ram_cursor(),
        cmd(16#26),
        insert_prev_frame(0),
        delay_ms(2),
        set_ram_cursor(),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        cmd(16#22, <<16#CF>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

full_window() ->
    [
        cmd(16#44, <<16#00, 16#18>>),
        cmd(16#45, <<16#C7, 16#00, 16#00, 16#00>>)
    ].

set_ram_cursor() ->
    [
        cmd(16#4E, <<16#00>>),
        cmd(16#4F, <<16#C7, 16#00>>)
    ].

wf_full_1in54() ->
    <<
        16#80, 16#48, 16#40, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#40, 16#48, 16#80, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#80, 16#48, 16#40, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#40, 16#48, 16#80, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#0A, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#08, 16#01, 16#00, 16#08, 16#01,
        16#00, 16#02, 16#0A, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#22, 16#22, 16#22, 16#22, 16#22, 16#22,
        16#00, 16#00, 16#00, 16#22, 16#17, 16#41,
        16#00, 16#32, 16#20
    >>.

wf_partial_1in54() ->
    <<
        16#00, 16#40, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#80, 16#80, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#40, 16#40, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#80, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#0F, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#01, 16#01, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#22, 16#22, 16#22, 16#22, 16#22, 16#22,
        16#00, 16#00, 16#00, 16#02, 16#17, 16#41,
        16#B0, 16#32, 16#28
    >>.
```

Use the descriptor exactly like a built-in one:

```erlang
{ok, Descriptor} = my_epaper_200x200:descriptor(),

Port = open_port({spawn, "display"}, [
    {descriptor, Descriptor},
    {spi_host, spi2},
    {cs, 5},
    {dc, 17},
    {reset, 16},
    {busy, 4}
]).
```

The GPIO numbers above are placeholders. Use the pins from your board wiring.

## Moving A Working Panel Into AtomGL

After hardware testing, move the descriptor into
`atomgl_epaper/src/atomgl_epaper_panels.erl`:

- Add a primary lookup string, such as `"waveshare,epd1in54_V2"`.
- Keep aliases or mode-default variants close to the primary clause.
- Keep copied vendor LUTs under an SPDX snippet notice.
- Add descriptor tests for controller, dimensions, refresh modes, and LUT byte
  sizes.
- Update `README.md` and `docs/display-drivers.md` with the new lookup string.
