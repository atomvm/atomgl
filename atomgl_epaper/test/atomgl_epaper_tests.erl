%% SPDX-License-Identifier: Apache-2.0
%% SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com>

-module(atomgl_epaper_tests).

-include_lib("eunit/include/eunit.hrl").

-import(atomgl_epaper, [panel/1, panel/2, validate_descriptor/1]).

-define(PROGRAM_DELAY, 16#80).
-define(PROGRAM_META, 16#40).
-define(PROGRAM_LEN_MASK, 16#3F).
-define(OP_INSERT_PLANE, 16#03).
-define(OP_INSERT_PREV_FRAME, 16#05).
-define(OP_CAPTURE_FRAME, 16#06).

get_value(Key, List) ->
    proplists:get_value(Key, List).

orientation_geometry_test_() ->
    [
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9_V2"),
            ?assertEqual(296, get_value(view_width, Desc)),
            ?assertEqual(128, get_value(view_height, Desc)),
            ?assertEqual(90, get_value(rotation, Desc))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9_V2", #{orientation => landscape_left}),
            ?assertEqual(296, get_value(view_width, Desc)),
            ?assertEqual(128, get_value(view_height, Desc)),
            ?assertEqual(90, get_value(rotation, Desc))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9_V2", #{orientation => landscape_right}),
            ?assertEqual(296, get_value(view_width, Desc)),
            ?assertEqual(128, get_value(view_height, Desc)),
            ?assertEqual(270, get_value(rotation, Desc))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9_V2", #{orientation => portrait}),
            ?assertEqual(128, get_value(view_width, Desc)),
            ?assertEqual(296, get_value(view_height, Desc)),
            ?assertEqual(0, get_value(rotation, Desc))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9_V2", #{orientation => portrait_flipped}),
            ?assertEqual(128, get_value(view_width, Desc)),
            ?assertEqual(296, get_value(view_height, Desc)),
            ?assertEqual(180, get_value(rotation, Desc))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9_V2", #{rotation => 0}),
            ?assertEqual(128, get_value(view_width, Desc)),
            ?assertEqual(296, get_value(view_height, Desc)),
            ?assertEqual(0, get_value(rotation, Desc))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9_V2",
                #{rotation => 0, view_width => 128, view_height => 296}),
            ?assertEqual(128, get_value(view_width, Desc)),
            ?assertEqual(296, get_value(view_height, Desc)),
            ?assertEqual(0, get_value(rotation, Desc))
        end)
    ].

invalid_rotation_descriptor_test() ->
    Desc = [
        {descriptor_version, 3},
        {controller, ssd16xx},
        {native_width, 128},
        {native_height, 296},
        {view_width, 128},
        {view_height, 296},
        {rotation, 45},
        {frame_layout, row_msb},
        {refresh_modes, [full]},
        {default_refresh, full},
        {programs, [{init, <<>>}, {full, <<>>}]}
    ],
    ?assertError({bad_rotation, 45}, validate_descriptor(Desc)).

invalid_rotated_view_geometry_test() ->
    Desc = [
        {descriptor_version, 3},
        {controller, ssd16xx},
        {native_width, 128},
        {native_height, 296},
        {view_width, 128},
        {view_height, 296},
        {rotation, 90},
        {frame_layout, row_msb},
        {refresh_modes, [full]},
        {default_refresh, full},
        {programs, [{init, <<>>}, {full, <<>>}]}
    ],
    ?assertError({bad_field, view_width, bad_geometry}, validate_descriptor(Desc)).

frame_size_limits_test_() ->
    Config = #{rotation => 90, init => <<0, 0>>, full => <<0, 0>>},
    [
        ?_test(begin
            {ok, Row} = atomgl_epaper_descriptor:ssd16xx_panel("row boundary",
                Config#{native_width => 32768, native_height => 1048575}),
            ?assertEqual(1048575, get_value(view_width, Row)),
            {ok, Column} = atomgl_epaper_descriptor:jd79656_panel("column boundary",
                Config#{native_width => 1048575, native_height => 32768}),
            ?assertEqual(1048575, get_value(view_height, Column))
        end),
        ?_test(begin
            %% 3 bytes per row times 1431655765 rows is exactly UINT32_MAX.
            {ok, _} = atomgl_epaper_descriptor:ssd16xx_panel("exact frame boundary",
                Config#{native_width => 24, native_height => 1431655765})
        end),
        ?_assertError({bad_field, frame_layout, frame_size_overflow},
            atomgl_epaper_descriptor:ssd16xx_panel("one row beyond frame boundary",
                Config#{native_width => 24, native_height => 1431655766})),
        ?_assertError({bad_field, frame_layout, frame_size_overflow},
            atomgl_epaper_descriptor:ssd16xx_panel("row overflow",
                Config#{native_width => 32768, native_height => 1048576})),
        ?_assertError({bad_field, frame_layout, frame_size_overflow},
            atomgl_epaper_descriptor:jd79656_panel("column overflow",
                Config#{native_width => 1048576, native_height => 32768})),
        %% Floor division would incorrectly accept these non-byte-aligned frames.
        ?_assertError({bad_field, frame_layout, frame_size_overflow},
            atomgl_epaper_descriptor:ssd16xx_panel("row rounding",
                Config#{native_width => 32769, native_height => 1048575})),
        ?_assertError({bad_field, frame_layout, frame_size_overflow},
            atomgl_epaper_descriptor:jd79656_panel("column rounding",
                Config#{native_width => 1048575, native_height => 32769})),
        ?_test(begin
            {ok, _} = atomgl_epaper_descriptor:ssd16xx_panel("dimension boundary",
                Config#{native_width => 2147483640, native_height => 1})
        end),
        ?_assertError({bad_field, native_width, {out_of_range, 1, 2147483640}},
            atomgl_epaper_descriptor:ssd16xx_panel("dimension rounding overflow",
                Config#{native_width => 2147483641, native_height => 1})),
        ?_assertError({bad_field, native_height, {out_of_range, 1, 2147483640}},
            atomgl_epaper_descriptor:jd79656_panel("dimension narrowing overflow",
                Config#{native_width => 1, native_height => 4294967297}))
    ].

invalid_refresh_modes_descriptor_test_() ->
    BaseDesc = [
        {descriptor_version, 3},
        {controller, ssd16xx},
        {native_width, 128},
        {native_height, 296},
        {view_width, 128},
        {view_height, 296},
        {rotation, 0},
        {frame_layout, row_msb},
        {polarity, white_1},
        {spi_clock_hz, 4000000},
        {busy_idle_level, 0},
        {use_gpio_pullups, false},
        {palette_size, 2},
        {default_refresh, full},
        {programs, [{init, <<0, 0>>}, {full, <<0, 0>>}, {partial, <<0, 0>>}]},
        {timing, []},
        {ghosting, []}
    ],
    [
        ?_test(begin
            ?assertError({bad_field, refresh_modes, duplicate},
                validate_descriptor([{refresh_modes, [full, partial, partial]} | BaseDesc]))
        end),
        ?_test(begin
            ?assertError({bad_field, refresh_modes, missing_full},
                validate_descriptor([{refresh_modes, [partial]} | BaseDesc]))
        end),
        ?_test(begin
            ?assertError({bad_field, refresh_modes, {expected_one_of, [full, fast, partial, '4gray']}},
                validate_descriptor([{refresh_modes, [full, turbo]} | BaseDesc]))
        end)
    ].

empty_required_program_test_() ->
    BaseDesc = [
        {descriptor_version, 3},
        {controller, ssd16xx},
        {native_width, 128},
        {native_height, 296},
        {view_width, 128},
        {view_height, 296},
        {rotation, 0},
        {frame_layout, row_msb},
        {polarity, white_1},
        {spi_clock_hz, 4000000},
        {busy_idle_level, 0},
        {use_gpio_pullups, false},
        {palette_size, 2},
        {refresh_modes, [full]},
        {default_refresh, full},
        {programs, [{init, <<0, 0>>}, {full, <<0, 0>>}]},
        {timing, []},
        {ghosting, []}
    ],
    [
        ?_assertError({bad_field, init, empty_program},
            validate_descriptor([{programs, [{init, <<>>}, {full, <<0, 0>>}]} | BaseDesc])),
        ?_assertError({bad_field, full, empty_program},
            validate_descriptor([{programs, [{init, <<0, 0>>}, {full, <<>>}]} | BaseDesc])),
        ?_assertError({bad_field, sleep, empty_program},
            validate_descriptor([{sleep_modes, [{sleep, [{enter, <<>>}]}]} | BaseDesc])),
        ?_assertError({bad_field, sleep, empty_program},
            validate_descriptor([{sleep_modes, [{sleep, [
                {enter, <<0, 0>>}, {wake, <<>>}
            ]}]} | BaseDesc])),
        ?_assertError({bad_field, init_seq, empty_sequence},
            panel("waveshare,5in65-acep-7c", #{init_seq => <<>>}))
    ].

invalid_previous_frame_flow_test_() ->
    {ok, Descriptor} = panel("waveshare,epd2in9_V2"),
    Programs = get_value(programs, Descriptor),
    [
        ?_assertError({bad_field, full, mark_without_capture},
            validate_descriptor(replace_value(programs,
                replace_value(full, <<16#07, 16#40>>, Programs), Descriptor))),
        ?_assertError({bad_field, full, capture_without_plane},
            validate_descriptor(replace_value(programs,
                replace_value(full, <<16#06, 16#40>>, Programs), Descriptor)))
    ].

fast_previous_frame_order_test_() ->
    [
        ?_test(assert_previous_frame_order("waveshare,epd2in9_V2")),
        ?_test(assert_previous_frame_order("waveshare,epd2in13_V4")),
        ?_test(assert_previous_frame_order("waveshare,epd4in2_V2"))
    ].

restricted_refresh_modes_test() ->
    {ok, Desc} = panel("waveshare,epd2in9_V2", #{refresh_modes => [full]}),
    ?assertEqual([full], get_value(refresh_modes, Desc)),
    ?assertEqual(full, get_value(default_refresh, Desc)),
    Programs = get_value(programs, Desc),
    ?assert(is_binary(get_value('4gray', Programs))),
    ?assertError({bad_field, fast, truncated_record},
        validate_descriptor(replace_value(programs,
            replace_value(fast, <<0>>, Programs), Desc))),
    ?assertError({bad_field, init, {bad_meta_opcode, 3, 1}},
        validate_descriptor(replace_value(programs,
            replace_value(init, <<3, 16#41, 0>>, Programs), Desc))).

heltec_busy_polarity_test() ->
    {ok, Desc} = panel("heltec,lcmen2r13efc1"),
    ?assertEqual(1, get_value(busy_idle_level, Desc)),
    Standby = get_value(standby, get_value(sleep_modes, Desc)),
    ?assertEqual(reset_init, get_value(wake, Standby)),
    {ok, LowBusyDesc} = panel("waveshare,epd2in9_V2"),
    ?assertEqual(0, get_value(busy_idle_level, LowBusyDesc)).

timing_options_test() ->
    {ok, Desc} = panel("waveshare,epd2in9_V2",
        #{timing => #{timeout_ms => 1234, poll_interval_ms => 25}}),
    Timing = get_value(timing, Desc),
    ?assertEqual(1234, get_value(timeout_ms, Timing)),
    ?assertEqual(25, get_value(poll_interval_ms, Timing)).

integer_field_limits_test() ->
    {ok, Desc} = panel("waveshare,epd2in9_V2", #{timeout_ms => 70000}),
    ?assertEqual(70000, get_value(timeout_ms, get_value(timing, Desc))),
    ?assertError({bad_field, timeout_ms, {out_of_range, 1, 2147483647}},
        panel("waveshare,epd2in9_V2", #{timeout_ms => 2147483648})),
    ?assertError({bad_field, max_fast_refreshes, {out_of_range, 0, 2147483647}},
        panel("waveshare,epd2in9_V2", #{max_fast_refreshes => 4294967296})).

uc8151_panel_descriptor_test_() ->
    [
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9"),
            ?assertEqual(uc8151, get_value(controller, Desc)),
            ?assertEqual(128, get_value(native_width, Desc)),
            ?assertEqual(296, get_value(native_height, Desc)),
            ?assertEqual([full, partial], get_value(refresh_modes, Desc)),
            ?assertEqual(30, byte_size(get_value(lut_full, Desc))),
            ?assertEqual(30, byte_size(get_value(lut_partial, Desc)))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in13"),
            ?assertEqual(uc8151, get_value(controller, Desc)),
            ?assertEqual(122, get_value(native_width, Desc)),
            ?assertEqual(250, get_value(native_height, Desc)),
            ?assertEqual(250, get_value(view_width, Desc)),
            ?assertEqual(122, get_value(view_height, Desc)),
            ?assertEqual(30, byte_size(get_value(lut_full, Desc))),
            ?assertEqual(30, byte_size(get_value(lut_partial, Desc)))
        end)
    ].

uc8276_panel_descriptor_test_() ->
    [
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd4in2_V2"),
            ?assertEqual(uc8276, get_value(controller, Desc)),
            ?assertEqual(400, get_value(native_width, Desc)),
            ?assertEqual(300, get_value(native_height, Desc)),
            ?assertEqual(300, get_value(view_width, Desc)),
            ?assertEqual(400, get_value(view_height, Desc)),
            ?assertEqual([full, fast, partial, '4gray'], get_value(refresh_modes, Desc)),
            ?assertEqual(233, byte_size(get_value(lut_4gray, Desc)))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd4in2_V2-4gray"),
            ?assertEqual(uc8276, get_value(controller, Desc)),
            ?assertEqual('4gray', get_value(default_refresh, Desc))
        end)
    ].

sleep_modes_descriptor_test_() ->
    [
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd2in9_V2"),
            ?assertEqual(3, get_value(descriptor_version, Desc)),
            SleepModes = get_value(sleep_modes, Desc),
            Sleep = get_value(sleep, SleepModes),
            DeepSleep = get_value(deep_sleep, SleepModes),
            ?assertEqual(<<16#10, 1, 16#01, 16#10, 16#C1, 0, 100>>,
                get_value(enter, Sleep)),
            ?assertEqual(retained, get_value(controller_ram, Sleep)),
            ?assertEqual(preserve, get_value(host_prev_frame, Sleep)),
            ?assertEqual(allow_if_program_reseeds, get_value(after_wake_refresh, Sleep)),
            ?assertEqual(<<16#10, 1, 16#03, 16#10, 16#C1, 0, 100>>,
                get_value(enter, DeepSleep)),
            ?assertEqual(lost, get_value(controller_ram, DeepSleep))
        end),
        ?_test(begin
            {ok, Desc} = panel("waveshare,epd4in2_V2"),
            SleepModes = get_value(sleep_modes, Desc),
            Sleep = get_value(sleep, SleepModes),
            ?assertEqual(<<16#10, 1, 16#01, 16#10, 16#C1, 0, 200>>,
                get_value(enter, Sleep))
        end),
        ?_test(begin
            {ok, Desc} = panel("good-display/gdep073e01"),
            SleepModes = get_value(sleep_modes, Desc),
            DeepSleep = get_value(deep_sleep, SleepModes),
            ?assertEqual(
                <<16#02, 1, 16#00, 16#01, 16#43, 1, 136, 19, 16#07, 1, 16#A5>>,
                get_value(enter, DeepSleep)),
            ?assertEqual(reset_init, get_value(wake, DeepSleep)),
            ?assertEqual(full, get_value(after_wake_refresh, DeepSleep))
        end),
        ?_test(begin
            CustomModes = [
                {lab, [
                    {enter, <<1, 2, 2, 3>>},
                    {wake, init},
                    {controller_ram, unknown},
                    {host_prev_frame, invalidate},
                    {after_wake_refresh, full}
                ]}
            ],
            {ok, Desc} = panel("waveshare,epd2in9_V2", #{sleep_modes => CustomModes}),
            ?assertEqual(CustomModes, get_value(sleep_modes, Desc))
        end),
        ?_test(begin
            DuplicateModes = [
                {sleep, [{enter, <<1>>}]},
                {sleep, [{enter, <<2>>}]}
            ],
            ?assertError({bad_field, sleep_modes, duplicate},
                panel("waveshare,epd2in9_V2", #{sleep_modes => DuplicateModes}))
        end),
        ?_test(begin
            TooManyModes = [
                {mode1, [{enter, <<1>>}]},
                {mode2, [{enter, <<2>>}]},
                {mode3, [{enter, <<3>>}]},
                {mode4, [{enter, <<4>>}]},
                {mode5, [{enter, <<5>>}]},
                {mode6, [{enter, <<6>>}]},
                {mode7, [{enter, <<7>>}]},
                {mode8, [{enter, <<8>>}]},
                {mode9, [{enter, <<9>>}]}
            ],
            ?assertError({bad_field, sleep_modes, too_many},
                panel("waveshare,epd2in9_V2", #{sleep_modes => TooManyModes}))
        end)
    ].

acep7_panel_descriptor_test_() ->
    [
        ?_test(begin
            {ok, Desc} = panel("waveshare,5in65-acep-7c"),
            ?assertEqual(acep7, get_value(controller, Desc)),
            ?assertEqual(600, get_value(native_width, Desc)),
            ?assertEqual(448, get_value(native_height, Desc)),
            ?assertEqual(acep7, get_value(palette, Desc)),
            ?assertEqual(46, byte_size(get_value(init_seq, Desc))),
            ?assertEqual(6, byte_size(get_value(frame_preamble_seq, Desc))),
            ?assertEqual(false, get_value(refresh_has_data, Desc)),
            ?assertEqual(5, get_value(periodic_refresh_interval, Desc))
        end),
        ?_test(begin
            {ok, Desc} = panel("good-display/gdep073e01"),
            ?assertEqual(acep7, get_value(controller, Desc)),
            ?assertEqual(800, get_value(native_width, Desc)),
            ?assertEqual(480, get_value(native_height, Desc)),
            ?assertEqual(gdep073e01, get_value(palette, Desc)),
            ?assertEqual(63, byte_size(get_value(init_seq, Desc))),
            ?assertEqual(true, get_value(init_wait_busy_between_cmds, Desc)),
            ?assertEqual(true, get_value(refresh_has_data, Desc)),
            ?assertEqual(1, get_value(post_power_off_busy_level, Desc))
        end),
        ?_test(begin
            ?assertError({bad_field, rotation, acep7_native_orientation_only},
                panel("waveshare,5in65-acep-7c",
                      #{rotation => 90, view_width => 448, view_height => 600}))
        end),
        ?_test(begin
            ?assertError({bad_field, init_seq, truncated_record},
                panel("waveshare,5in65-acep-7c", #{init_seq => <<0>>}))
        end),
        ?_test(begin
            ?assertError({bad_field, native_width, acep7_width_must_be_even},
                panel("waveshare,5in65-acep-7c", #{native_width => 601}))
        end)
    ].

unknown_panel_test() ->
    ?assertEqual({error, {unsupported_panel, "missing,panel"}},
        panel("missing,panel")).

assert_previous_frame_order(Compatible) ->
    {ok, Descriptor} = panel(Compatible),
    Programs = get_value(programs, Descriptor),
    FastOps = program_meta_opcodes(get_value(fast, Programs)),
    ?assert(opcode_index(?OP_INSERT_PREV_FRAME, FastOps)
        < opcode_index(?OP_CAPTURE_FRAME, FastOps)),
    ?assert(opcode_index(?OP_CAPTURE_FRAME, FastOps)
        < opcode_index(?OP_INSERT_PLANE, FastOps)),
    FullOps = program_meta_opcodes(get_value(full, Programs)),
    ?assert(opcode_index(?OP_CAPTURE_FRAME, FullOps)
        < opcode_index(?OP_INSERT_PLANE, FullOps)),
    ?assert(opcode_index(?OP_INSERT_PLANE, FullOps)
        < opcode_index(?OP_INSERT_PREV_FRAME, FullOps)).

program_meta_opcodes(<<>>) ->
    [];
program_meta_opcodes(<<Opcode, FlagsLen, Rest/binary>>) ->
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
    case IsMeta of
        true -> [Opcode | program_meta_opcodes(Tail)];
        false -> program_meta_opcodes(Tail)
    end.

opcode_index(Opcode, Opcodes) ->
    opcode_index(Opcode, Opcodes, 1).

opcode_index(Opcode, [Opcode | _], Index) ->
    Index;
opcode_index(Opcode, [_ | Rest], Index) ->
    opcode_index(Opcode, Rest, Index + 1).

replace_value(Key, Value, Proplist) ->
    [{Key, Value} | proplists:delete(Key, Proplist)].
