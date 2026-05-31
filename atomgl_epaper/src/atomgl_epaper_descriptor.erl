%% SPDX-License-Identifier: Apache-2.0
%% SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com>

-module(atomgl_epaper_descriptor).

-define(MAX_SLEEP_MODES, 8).
-define(MAX_DESCRIPTOR_BINARY_LEN, 4096).
-define(MAX_INT, 16#7FFFFFFF).
-define(MAX_DIMENSION, (?MAX_INT - 7)).
-define(MAX_FRAME_BYTES, 16#FFFFFFFF).
-define(PROGRAM_DELAY, 16#80).
-define(PROGRAM_META, 16#40).
-define(PROGRAM_LEN_MASK, 16#3F).
-define(INIT_SEQ_DELAY, 16#80).
-define(INIT_SEQ_LEN_MASK, 16#7F).

-define(OP_WAIT_BUSY, 16#01).
-define(OP_RESET, 16#02).
-define(OP_INSERT_PLANE, 16#03).
-define(OP_INSERT_LUT, 16#04).
-define(OP_INSERT_PREV_FRAME, 16#05).
-define(OP_CAPTURE_FRAME, 16#06).
-define(OP_MARK_PREV_VALID, 16#07).
-define(OP_LABEL, 16#10).

-export([
    ssd16xx_panel/2,
    jd79656_panel/2,
    uc8151_panel/2,
    uc8276_panel/2,
    acep7_panel/2,
    validate_descriptor/1
]).

ssd16xx_panel(Name, Config) ->
    build_panel(ssd16xx, Name, Config).

jd79656_panel(Name, Config) ->
    build_panel(jd79656, Name, Config).

uc8151_panel(Name, Config) ->
    build_panel(uc8151, Name, Config).

uc8276_panel(Name, Config) ->
    build_panel(uc8276, Name, Config).

acep7_panel(Name, Config) ->
    NativeW = maps:get(native_width, Config),
    NativeH = maps:get(native_height, Config),
    Rotation = maps:get(rotation, Config, 0),
    ViewW = maps:get(view_width, Config, NativeW),
    ViewH = maps:get(view_height, Config, NativeH),

    Descriptor = [
        {descriptor_version, 3},
        {name, Name},
        {controller, acep7},
        {native_width, NativeW},
        {native_height, NativeH},
        {view_width, ViewW},
        {view_height, ViewH},
        {rotation, Rotation},
        {spi_clock_hz, maps:get(spi_clock_hz, Config, 4000000)},
        {busy_idle_level, maps:get(busy_idle_level, Config, 1)},
        {use_gpio_pullups, maps:get(use_gpio_pullups, Config, true)},
        {frame_layout, maps:get(frame_layout, Config, row_msb)},
        {polarity, maps:get(polarity, Config, white_1)},
        {refresh_modes, maps:get(refresh_modes, Config, [full])},
        {default_refresh, maps:get(default_refresh, Config, full)},
        {palette, maps:get(palette, Config)},
        {palette_size, maps:get(palette_size, Config, 7)},
        {init_seq, maps:get(init_seq, Config)},
        {init_wait_busy_between_cmds, maps:get(init_wait_busy_between_cmds, Config, false)},
        {refresh_has_data, maps:get(refresh_has_data, Config, false)},
        {refresh_data_byte, maps:get(refresh_data_byte, Config, 0)},
        {post_power_off_busy_level, maps:get(post_power_off_busy_level, Config, 0)},
        {periodic_refresh_interval, maps:get(periodic_refresh_interval, Config, 0)},
        {sleep_modes, maps:get(sleep_modes, Config, [])}
    ] ++ case maps:find(frame_preamble_seq, Config) of
        {ok, FramePreambleSeq} -> [{frame_preamble_seq, FramePreambleSeq}];
        error -> []
    end,

    validate_descriptor(Descriptor).

build_panel(Controller, Name, Config) ->
    NativeW = maps:get(native_width, Config),
    NativeH = maps:get(native_height, Config),
    OrientationRotation = orientation_rotation(maps:get(orientation, Config, landscape)),
    Rotation = maps:get(rotation, Config, OrientationRotation),
    {DefaultViewW, DefaultViewH} = default_view_geometry(NativeW, NativeH, Rotation),
    ViewW = maps:get(view_width, Config, DefaultViewW),
    ViewH = maps:get(view_height, Config, DefaultViewH),

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
        uc8151 -> {row_msb, white_1};
        uc8276 -> {row_msb, white_1};
        uc8175 -> {row_msb, white_1}
    end,

    Descriptor = [
        {descriptor_version, 3},
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
        {sleep_modes, maps:get(sleep_modes, Config, [])},
        {timing, Timing},
        {ghosting, Ghosting}
    ] ++ [L || L = {_, Bin} <- Luts, Bin =/= <<>>],

    validate_descriptor(Descriptor).

orientation_rotation(landscape) ->
    90;
orientation_rotation(landscape_left) ->
    90;
orientation_rotation(landscape_right) ->
    270;
orientation_rotation(portrait) ->
    0;
orientation_rotation(portrait_flipped) ->
    180;
orientation_rotation(Orientation) ->
    error({bad_orientation, Orientation}).

default_view_geometry(NativeW, NativeH, Rotation) ->
    case Rotation of
        0 -> {NativeW, NativeH};
        180 -> {NativeW, NativeH};
        90 -> {NativeH, NativeW};
        270 -> {NativeH, NativeW};
        _ -> {NativeW, NativeH}
    end.

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
    P5 = case maps:find('4gray', Config) of
        {ok, FourGray} -> [{'4gray', FourGray} | P4];
        error -> P4
    end,
    P5.

timing(Config) ->
    TimingConfig = maps:get(timing, Config, #{}),
    [
        {full_expected_ms, config_value(full_expected_ms, TimingConfig, Config, 2000)},
        {fast_expected_ms, config_value(fast_expected_ms, TimingConfig, Config, 500)},
        {poll_interval_ms, config_value(poll_interval_ms, TimingConfig, Config, 50)},
        {timeout_ms, config_value(timeout_ms, TimingConfig, Config, 5000)}
    ].

config_value(Key, Nested, Config, Default) when is_map(Nested) ->
    maps:get(Key, Nested, maps:get(Key, Config, Default));
config_value(Key, _Nested, Config, Default) ->
    maps:get(Key, Config, Default).

validate_descriptor(Desc) when is_list(Desc) ->
    Version = get_value(descriptor_version, Desc),
    validate_int(descriptor_version, Version, 3, 3),

    NativeW = get_value(native_width, Desc),
    NativeH = get_value(native_height, Desc),
    ViewW = get_value(view_width, Desc),
    ViewH = get_value(view_height, Desc),
    Rotation = get_value(rotation, Desc),
    validate_int(native_width, NativeW, 1, ?MAX_DIMENSION),
    validate_int(native_height, NativeH, 1, ?MAX_DIMENSION),
    validate_int(view_width, ViewW, 1, ?MAX_DIMENSION),
    validate_int(view_height, ViewH, 1, ?MAX_DIMENSION),
    validate_int(rotation, Rotation, 0, 270),

    case Rotation of
        0 ->
            require_true(view_width, ViewW == NativeW, bad_geometry),
            require_true(view_height, ViewH == NativeH, bad_geometry);
        180 ->
            require_true(view_width, ViewW == NativeW, bad_geometry),
            require_true(view_height, ViewH == NativeH, bad_geometry);
        90 ->
            require_true(view_width, ViewW == NativeH, bad_geometry),
            require_true(view_height, ViewH == NativeW, bad_geometry);
        270 ->
            require_true(view_width, ViewW == NativeH, bad_geometry),
            require_true(view_height, ViewH == NativeW, bad_geometry);
        _ ->
            error({bad_rotation, Rotation})
    end,

    Controller = get_value(controller, Desc),
    validate_member(controller, Controller, [ssd16xx, jd79656, uc8151, uc8276, uc8175, acep7]),

    Layout = get_value(frame_layout, Desc),
    require_true(frame_layout, case Controller of
        ssd16xx -> lists:member(Layout, [row_msb, row_lsb]);
        uc8151 -> lists:member(Layout, [row_msb, row_lsb]);
        uc8276 -> lists:member(Layout, [row_msb, row_lsb]);
        uc8175 -> lists:member(Layout, [row_msb, row_lsb]);
        jd79656 -> lists:member(Layout, [column_msb, column_lsb]);
        _ -> true
    end, incompatible_with_controller),
    case Controller of
        acep7 -> ok;
        _ ->
            FrameBytes = case Layout of
                column_msb -> NativeW * ((NativeH + 7) div 8);
                column_lsb -> NativeW * ((NativeH + 7) div 8);
                _ -> ((NativeW + 7) div 8) * NativeH
            end,
            require_true(frame_layout, FrameBytes =< ?MAX_FRAME_BYTES,
                frame_size_overflow)
    end,
    validate_member(polarity, get_value(polarity, Desc), [white_1, black_1]),
    validate_int(spi_clock_hz, get_value(spi_clock_hz, Desc), 1, 80000000),
    validate_member(busy_idle_level, get_value(busy_idle_level, Desc), [0, 1]),
    validate_bool(use_gpio_pullups, get_value(use_gpio_pullups, Desc)),

    DefaultRefresh = get_value(default_refresh, Desc),
    RefreshModes = get_value(refresh_modes, Desc),
    validate_refresh_modes(RefreshModes),
    validate_member(default_refresh, DefaultRefresh, RefreshModes),
    validate_sleep_modes(get_value(sleep_modes, Desc, [])),
    PaletteSize = get_value(palette_size, Desc),
    validate_member(palette_size, PaletteSize, [2, 4, 7]),
    validate_timing(get_value(timing, Desc, [])),
    validate_ghosting(get_value(ghosting, Desc, [])),

    case Controller of
        acep7 ->
            require_true(rotation, Rotation == 0, acep7_native_orientation_only),
            require_true(native_width, (NativeW rem 2) == 0, acep7_width_must_be_even),
            require_true(view_width, (ViewW rem 2) == 0, acep7_width_must_be_even),
            validate_member(palette_size, PaletteSize, [7]),
            Palette = get_value(palette, Desc),
            validate_member(palette, Palette, [acep7, acep7c, gdep073e01]),
            InitSeq = get_value(init_seq, Desc),
            validate_required_init_seq(init_seq, InitSeq),
            FramePreambleSeq = get_value(frame_preamble_seq, Desc, <<>>),
            validate_init_seq(frame_preamble_seq, FramePreambleSeq),
            validate_bool(init_wait_busy_between_cmds, get_value(init_wait_busy_between_cmds, Desc)),
            validate_bool(refresh_has_data, get_value(refresh_has_data, Desc)),
            validate_int(refresh_data_byte, get_value(refresh_data_byte, Desc), 0, 255),
            validate_member(post_power_off_busy_level,
                get_value(post_power_off_busy_level, Desc), [0, 1]),
            validate_int(periodic_refresh_interval,
                get_value(periodic_refresh_interval, Desc), 0, ?MAX_INT),
            validate_member(refresh_modes, RefreshModes, [[full]]);
        _ ->
            require_true(palette_size, PaletteSize =/= 7, acep7_only),
            require_true(palette_size,
                PaletteSize =/= 4 orelse lists:member(Controller, [ssd16xx, uc8276]),
                unsupported_4gray_controller),
            require_true(palette_size,
                not lists:member('4gray', RefreshModes) orelse PaletteSize == 4,
                required_for_4gray),
            Programs = get_value(programs, Desc),
            validate_kv_list(programs, Programs),
            require_true(programs, is_defined(init, Programs), missing_init),

            lists:foreach(fun(Mode) ->
                require_true(programs, is_defined(Mode, Programs),
                    {missing_refresh_program, Mode})
            end, RefreshModes),

            validate_required_program(init, get_value(init, Programs), false),
            lists:foreach(fun(Mode) ->
                validate_required_program(Mode, get_value(Mode, Programs), true)
            end, RefreshModes),
            lists:foreach(fun({Mode, Program}) ->
                validate_program(Mode, Program,
                    lists:member(Mode, [full, fast, partial, '4gray']))
            end, Programs),
            validate_optional_binary(lut_full, get_value(lut_full, Desc, <<>>)),
            validate_optional_binary(lut_partial, get_value(lut_partial, Desc, <<>>)),
            validate_optional_binary(lut_4gray, get_value(lut_4gray, Desc, <<>>)),
            validate_optional_binary(lut_fast, get_value(lut_fast, Desc, <<>>))
    end,

    {ok, Desc};
validate_descriptor(_) ->
    error({bad_descriptor, expected_keyword_list}).

bad_field(Key, Reason) ->
    error({bad_field, Key, Reason}).

require_true(_Key, true, _Reason) ->
    ok;
require_true(Key, false, Reason) ->
    bad_field(Key, Reason).

validate_int(Key, Value, Min, Max) when is_integer(Value), Value >= Min ->
    case Max of
        infinity -> ok;
        _ when Value =< Max -> ok;
        _ -> bad_field(Key, {out_of_range, Min, Max})
    end;
validate_int(Key, _Value, Min, Max) ->
    bad_field(Key, {expected_integer, Min, Max}).

validate_bool(_Key, Value) when is_boolean(Value) ->
    ok;
validate_bool(Key, _Value) ->
    bad_field(Key, expected_boolean).

validate_member(_Key, Value, Allowed) ->
    case lists:member(Value, Allowed) of
        true -> ok;
        false -> bad_field(_Key, {expected_one_of, Allowed})
    end.

validate_kv_list(Key, Value) when is_list(Value) ->
    lists:foreach(fun
        ({K, _V}) when is_atom(K) -> ok;
        (_) -> bad_field(Key, expected_keyword_list)
    end, Value);
validate_kv_list(Key, _Value) ->
    bad_field(Key, expected_keyword_list).

validate_refresh_modes(RefreshModes) when is_list(RefreshModes) ->
    require_true(refresh_modes, RefreshModes =/= [], empty),
    require_true(refresh_modes, lists:member(full, RefreshModes), missing_full),
    require_true(refresh_modes,
        length(RefreshModes) == length(lists:usort(RefreshModes)), duplicate),
    lists:foreach(fun(Mode) ->
        validate_member(refresh_modes, Mode, [full, fast, partial, '4gray'])
    end, RefreshModes);
validate_refresh_modes(_) ->
    bad_field(refresh_modes, expected_list).

validate_timing(Timing) ->
    validate_kv_list(timing, Timing),
    validate_int(full_expected_ms, get_value(full_expected_ms, Timing, 2000), 0, ?MAX_INT),
    validate_int(fast_expected_ms, get_value(fast_expected_ms, Timing, 500), 0, ?MAX_INT),
    validate_int(poll_interval_ms, get_value(poll_interval_ms, Timing, 50), 1, ?MAX_INT),
    validate_int(timeout_ms, get_value(timeout_ms, Timing, 5000), 1, ?MAX_INT).

validate_ghosting(Ghosting) ->
    validate_kv_list(ghosting, Ghosting),
    validate_int(max_fast_refreshes, get_value(max_fast_refreshes, Ghosting, 10), 0, ?MAX_INT),
    validate_bool(reseed_on_timeout, get_value(reseed_on_timeout, Ghosting, true)).

validate_optional_binary(_Key, <<>>) ->
    ok;
validate_optional_binary(_Key, Bin) when is_binary(Bin), byte_size(Bin) =< ?MAX_DESCRIPTOR_BINARY_LEN ->
    ok;
validate_optional_binary(Key, _Value) ->
    bad_field(Key, expected_binary).

validate_program(Key, Bin, AllowRenderOps) when is_binary(Bin),
        byte_size(Bin) =< ?MAX_DESCRIPTOR_BINARY_LEN ->
    validate_program_bytes(Key, Bin, AllowRenderOps),
    validate_program_flow(Key, Bin, false, false);
validate_program(Key, _Value, _AllowRenderOps) ->
    bad_field(Key, expected_program_binary).

validate_required_program(Key, <<>>, _AllowRenderOps) ->
    bad_field(Key, empty_program);
validate_required_program(Key, Bin, AllowRenderOps) ->
    validate_program(Key, Bin, AllowRenderOps).

validate_program_bytes(_Key, <<>>, _AllowRenderOps) ->
    ok;
validate_program_bytes(Key, <<Opcode, FlagsLen, Rest/binary>>, AllowRenderOps) ->
    Len = FlagsLen band ?PROGRAM_LEN_MASK,
    HasDelay = (FlagsLen band ?PROGRAM_DELAY) =/= 0,
    IsMeta = (FlagsLen band ?PROGRAM_META) =/= 0,
    case Rest of
        <<Data:Len/binary, Tail/binary>> ->
            case IsMeta of
                true -> validate_meta(Key, Opcode, byte_size(Data), AllowRenderOps);
                false -> ok
            end,
            case {HasDelay, Tail} of
                {true, <<_Delay, Next/binary>>} ->
                    validate_program_bytes(Key, Next, AllowRenderOps);
                {true, _} ->
                    bad_field(Key, truncated_delay);
                {false, _} ->
                    validate_program_bytes(Key, Tail, AllowRenderOps)
            end;
        _ ->
            bad_field(Key, truncated_record)
    end;
validate_program_bytes(Key, _Bad, _AllowRenderOps) ->
    bad_field(Key, truncated_record).

validate_program_flow(Key, <<>>, true, _CaptureCompleted) ->
    bad_field(Key, capture_without_plane);
validate_program_flow(_Key, <<>>, false, _CaptureCompleted) ->
    ok;
validate_program_flow(Key, <<Opcode, FlagsLen, Rest/binary>>,
        CapturePending, CaptureCompleted) ->
    Len = FlagsLen band ?PROGRAM_LEN_MASK,
    HasDelay = (FlagsLen band ?PROGRAM_DELAY) =/= 0,
    IsMeta = (FlagsLen band ?PROGRAM_META) =/= 0,
    <<_Data:Len/binary, Tail0/binary>> = Rest,
    Tail = case HasDelay of
        true ->
            <<_Delay, Next/binary>> = Tail0,
            Next;
        false ->
            Tail0
    end,
    {NextPending, NextCompleted} = case {IsMeta, Opcode, CapturePending} of
        {true, ?OP_CAPTURE_FRAME, true} ->
            bad_field(Key, capture_already_pending);
        {true, ?OP_CAPTURE_FRAME, false} ->
            {true, CaptureCompleted};
        {true, ?OP_INSERT_PLANE, true} ->
            {false, true};
        {true, ?OP_MARK_PREV_VALID, _} when not CaptureCompleted ->
            bad_field(Key, mark_without_capture);
        _ ->
            {CapturePending, CaptureCompleted}
    end,
    validate_program_flow(Key, Tail, NextPending, NextCompleted).

validate_meta(_Key, ?OP_WAIT_BUSY, 3, _AllowRenderOps) -> ok;
validate_meta(_Key, ?OP_RESET, 3, _AllowRenderOps) -> ok;
validate_meta(_Key, ?OP_INSERT_LUT, 1, _AllowRenderOps) -> ok;
validate_meta(_Key, ?OP_LABEL, 1, _AllowRenderOps) -> ok;
validate_meta(_Key, ?OP_INSERT_PLANE, 1, true) -> ok;
validate_meta(_Key, ?OP_INSERT_PREV_FRAME, 1, true) -> ok;
validate_meta(_Key, ?OP_CAPTURE_FRAME, 0, true) -> ok;
validate_meta(_Key, ?OP_MARK_PREV_VALID, 0, true) -> ok;
validate_meta(Key, Opcode, Len, _AllowRenderOps) ->
    bad_field(Key, {bad_meta_opcode, Opcode, Len}).

validate_init_seq(Key, Bin) when is_binary(Bin),
        byte_size(Bin) =< ?MAX_DESCRIPTOR_BINARY_LEN ->
    validate_init_seq_bytes(Key, Bin);
validate_init_seq(Key, _Value) ->
    bad_field(Key, expected_init_sequence_binary).

validate_required_init_seq(Key, <<>>) ->
    bad_field(Key, empty_sequence);
validate_required_init_seq(Key, Bin) ->
    validate_init_seq(Key, Bin).

validate_init_seq_bytes(_Key, <<>>) ->
    ok;
validate_init_seq_bytes(Key, <<_Cmd, FlagsLen, Rest/binary>>) ->
    Len = FlagsLen band ?INIT_SEQ_LEN_MASK,
    HasDelay = (FlagsLen band ?INIT_SEQ_DELAY) =/= 0,
    case Rest of
        <<_Data:Len/binary, Tail/binary>> ->
            case {HasDelay, Tail} of
                {true, <<_Delay, Next/binary>>} ->
                    validate_init_seq_bytes(Key, Next);
                {true, _} ->
                    bad_field(Key, truncated_delay);
                {false, _} ->
                    validate_init_seq_bytes(Key, Tail)
            end;
        _ ->
            bad_field(Key, truncated_record)
    end;
validate_init_seq_bytes(Key, _Bad) ->
    bad_field(Key, truncated_record).

validate_sleep_modes(SleepModes) when is_list(SleepModes) ->
    require_true(sleep_modes, length(SleepModes) =< ?MAX_SLEEP_MODES, too_many),
    Modes = lists:map(fun
        ({Mode, _Props}) when is_atom(Mode) -> Mode;
        (_) -> bad_field(sleep_modes, bad_sleep_mode)
    end, SleepModes),
    require_true(sleep_modes, length(Modes) == length(lists:usort(Modes)), duplicate),
    lists:foreach(fun validate_sleep_mode/1, SleepModes);
validate_sleep_modes(_) ->
    bad_field(sleep_modes, expected_list).

validate_sleep_mode({Mode, Props}) when is_atom(Mode), is_list(Props) ->
    validate_kv_list(Mode, Props),
    Enter = get_value(enter, Props),
    validate_required_program(Mode, Enter, false),
    Wake = get_value(wake, Props, reset_init),
    case Wake of
        Bin when is_binary(Bin) ->
            validate_required_program(Mode, Bin, false);
        _ ->
            validate_member(Mode, Wake, [init, reset_init])
    end,
    validate_member(Mode, get_value(controller_ram, Props, unknown),
        [unknown, retained, lost]),
    validate_member(Mode, get_value(host_prev_frame, Props, invalidate),
        [preserve, invalidate]),
    validate_member(Mode, get_value(after_wake_refresh, Props, full),
        [allow, full, allow_if_program_reseeds]);
validate_sleep_mode(_) ->
    bad_field(sleep_modes, bad_sleep_mode).

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
