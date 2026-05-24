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
