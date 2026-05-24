%% SPDX-License-Identifier: Apache-2.0
%% SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com>

-module(atomgl_epaper_descriptor).

-export([
    ssd16xx_panel/2,
    jd79656_panel/2,
    validate_descriptor/1
]).

ssd16xx_panel(Name, Config) ->
    build_panel(ssd16xx, Name, Config).

jd79656_panel(Name, Config) ->
    build_panel(jd79656, Name, Config).

build_panel(Controller, Name, Config) ->
    NativeW = maps:get(native_width, Config),
    NativeH = maps:get(native_height, Config),
    Orientation = maps:get(orientation, Config, landscape),
    {ViewW, ViewH, Rotation} = case Orientation of
        landscape -> {NativeH, NativeW, 90};
        landscape_left -> {NativeH, NativeW, 90};
        landscape_right -> {NativeH, NativeW, 270};
        portrait -> {NativeW, NativeH, 0};
        portrait_flipped -> {NativeW, NativeH, 180};
        _ -> error({bad_orientation, Orientation})
    end,

    DefaultRefresh = maps:get(default_refresh, Config, full),
    RefreshModes = maps:get(refresh_modes, Config, [full]),
    PaletteSize = maps:get(palette_size, Config, 2),

    Luts = [
        {lut_full, maps:get(lut_full, Config, <<>>)},
        {lut_partial, maps:get(lut_partial, Config, <<>>)},
        {lut_4gray, maps:get(lut_4gray, Config, <<>>)},
        {lut_fast, maps:get(lut_fast, Config, <<>>)}
    ],

    Programs = mode_programs(Config),
    Timing = timing(Config),
    GhostingConfig = maps:get(ghosting, Config, #{}),
    Ghosting = [
        {max_fast_refreshes, maps:get(max_fast_refreshes, GhostingConfig, maps:get(max_fast_refreshes, Config, 10))},
        {reseed_on_timeout, maps:get(reseed_on_timeout, GhostingConfig, maps:get(reseed_on_timeout, Config, true))}
    ],

    {Layout, Polarity} = case Controller of
        ssd16xx -> {row_msb, white_1};
        jd79656 -> {column_msb, black_1};
        uc8175 -> {row_msb, white_1}
    end,

    Descriptor = [
        {descriptor_version, 2},
        {name, Name},
        {controller, Controller},
        {native_width, NativeW},
        {native_height, NativeH},
        {view_width, ViewW},
        {view_height, ViewH},
        {rotation, Rotation},
        {spi_clock_hz, maps:get(spi_clock_hz, Config, 4000000)},
        {busy_idle_level, maps:get(busy_idle_level, Config, 0)},
        {use_gpio_pullups, maps:get(use_gpio_pullups, Config, false)},
        {frame_layout, Layout},
        {polarity, Polarity},
        {refresh_modes, RefreshModes},
        {default_refresh, DefaultRefresh},
        {palette_size, PaletteSize},
        {programs, Programs},
        {timing, Timing},
        {ghosting, Ghosting}
    ] ++ [L || L = {_, Bin} <- Luts, Bin =/= <<>>],

    validate_descriptor(Descriptor).

mode_programs(Config) ->
    P0 = [],
    P1 = case maps:find(init, Config) of
        {ok, Init} -> [{init, Init} | P0];
        error -> P0
    end,
    P2 = case maps:find(full, Config) of
        {ok, Full} -> [{full, Full} | P1];
        error -> P1
    end,
    P3 = case maps:find(fast, Config) of
        {ok, Fast} -> [{fast, Fast} | P2];
        error -> P2
    end,
    P4 = case maps:find(partial, Config) of
        {ok, Partial} -> [{partial, Partial} | P3];
        error -> P3
    end,
    P5 = case maps:find(sleep, Config) of
        {ok, Sleep} -> [{sleep, Sleep} | P4];
        error -> P4
    end,
    P6 = case maps:find('4gray', Config) of
        {ok, FourGray} -> [{'4gray', FourGray} | P5];
        error -> P5
    end,
    P6.

timing(Config) ->
    [
        {full_expected_ms, maps:get(full_expected_ms, Config, 2000)},
        {fast_expected_ms, maps:get(fast_expected_ms, Config, 500)},
        {poll_interval_ms, maps:get(poll_interval_ms, Config, 50)},
        {timeout_ms, maps:get(timeout_ms, Config, 5000)}
    ].

validate_descriptor(Desc) ->
    Version = get_value(descriptor_version, Desc),
    true = (Version == 2),

    NativeW = get_value(native_width, Desc),
    NativeH = get_value(native_height, Desc),
    ViewW = get_value(view_width, Desc),
    ViewH = get_value(view_height, Desc),
    Rotation = get_value(rotation, Desc),

    %% Width/height consistency check
    case Rotation of
        0 ->
            true = (ViewW == NativeW),
            true = (ViewH == NativeH);
        180 ->
            true = (ViewW == NativeW),
            true = (ViewH == NativeH);
        90 ->
            true = (ViewW == NativeH),
            true = (ViewH == NativeW);
        270 ->
            true = (ViewW == NativeH),
            true = (ViewH == NativeW);
        _ ->
            error({bad_rotation, Rotation})
    end,

    Controller = get_value(controller, Desc),
    true = lists:member(Controller, [ssd16xx, jd79656, uc8175, acep7]),

    Layout = get_value(frame_layout, Desc),
    true = case Controller of
        ssd16xx -> lists:member(Layout, [row_msb, row_lsb]);
        uc8175 -> lists:member(Layout, [row_msb, row_lsb]);
        jd79656 -> lists:member(Layout, [column_msb, column_lsb]);
        _ -> true
    end,

    DefaultRefresh = get_value(default_refresh, Desc),
    RefreshModes = get_value(refresh_modes, Desc),
    true = lists:member(DefaultRefresh, RefreshModes),

    Programs = get_value(programs, Desc),
    true = is_defined(init, Programs),

    %% Required programs for declared modes
    lists:foreach(fun(Mode) ->
        true = is_defined(Mode, Programs)
    end, RefreshModes),

    %% Bytecode payload length limits
    lists:foreach(fun({_K, V}) ->
        true = (byte_size(V) =< 4096)
    end, Programs),

    {ok, Desc}.

get_value(Key, List) ->
    get_value(Key, List, undefined).

get_value(Key, [{Key, Val} | _], _Default) ->
    Val;
get_value(Key, [_ | Tail], Default) ->
    get_value(Key, Tail, Default);
get_value(_Key, [], Default) ->
    Default.

is_defined(Key, [{Key, _} | _]) ->
    true;
is_defined(Key, [_ | Tail]) ->
    is_defined(Key, Tail);
is_defined(_Key, []) ->
    false.
