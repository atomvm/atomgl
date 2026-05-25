%% SPDX-License-Identifier: Apache-2.0
%% SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com>

-module(atomgl_epaper_tests).

-include_lib("eunit/include/eunit.hrl").

-import(atomgl_epaper, [panel/1, panel/2, validate_descriptor/1]).

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
    ?assertError({badmatch, false}, validate_descriptor(Desc)).

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
        {default_refresh, full},
        {programs, [{init, <<>>}, {full, <<>>}, {partial, <<>>}]}
    ],
    [
        ?_test(begin
            ?assertError({badmatch, false},
                validate_descriptor([{refresh_modes, [full, partial, partial]} | BaseDesc]))
        end),
        ?_test(begin
            ?assertError({badmatch, false},
                validate_descriptor([{refresh_modes, [partial]} | BaseDesc]))
        end),
        ?_test(begin
            ?assertError({badmatch, false},
                validate_descriptor([{refresh_modes, [full, turbo]} | BaseDesc]))
        end)
    ].

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
                    {enter, <<1, 2, 3>>},
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
            ?assertError({badmatch, false},
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
            ?assertError({badmatch, false},
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
            ?assertError({badmatch, false},
                panel("waveshare,5in65-acep-7c",
                      #{rotation => 90, view_width => 448, view_height => 600}))
        end)
    ].
