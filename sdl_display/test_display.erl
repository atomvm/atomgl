%
% This file is part of AtomVM.
%
% Copyright 2023 Davide Bettio <davide@uninstall.it>
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%    http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%
% SPDX-License-Identifier: Apache-2.0
%

-module(test_display).
-export([start/0, shapes/0, loop/0]).

start() ->
    Display = erlang:open_port({spawn, "display"}, []),
    disp(Display, 16#00000000),
    disp(Display, 16#000000FF),
    disp(Display, 16#00FF0000),
    disp(Display, 16#0000FF00),

    Display ! {'$call', {self(), make_ref()}, {subscribe_input, all}},

    loop().

shapes() ->
    Display = erlang:open_port({spawn, "display"}, []),
    Sprite = {rgba8888, 2, 1, <<255, 0, 0, 255, 0, 0, 255, 255>>},
    Scene = [
        {text, 10, 4, default16px, 16#FFFFFF, transparent, <<"shapes">>},
        {rounded_rect, 10, 24, 100, 30, 8, 16#3060C0},
        {circle, 40, 90, 20, 16#20C040},
        {ellipse, 110, 90, 35, 15, 16#C04080},
        {scaled_cropped_image, 110, 140, 40, 20, transparent, 0, 0, 20, 20, [], Sprite},
        {scaled_cropped_image, 160, 140, 40, 20, transparent, 0, 0, 20, 20, [{flip_x, true}], Sprite},
        {rect, 0, 0, 240, 240, 16#202020}
    ],
    Display ! {'$call', {self(), make_ref()}, {update, Scene}},
    loop().

disp(Display, Num) ->
    Bin = integer_to_binary(Num),
    Scene = [
        {text, 10, 20, default16px, Num, 16#FFFFFF, <<"Test ", Bin/binary>>}
    ],
    Display ! {'$call', {self(), make_ref()}, {update, Scene}}.

loop() ->
    receive
        Any -> erlang:display(Any)
    end,
    loop().
