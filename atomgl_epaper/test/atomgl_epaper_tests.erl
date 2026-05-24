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
        {descriptor_version, 2},
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
        {descriptor_version, 2},
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
