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
% SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
%

-module(test_display).
-export([start/0, loop/0]).

start() ->
    Display = erlang:open_port({spawn, "display"}, []),
    disp(Display, 16#00000000),
    disp(Display, 16#000000FF),
    disp(Display, 16#00FF0000),
    disp(Display, 16#0000FF00),

    % TODO: let's switch back to the following as soon as tuple format is changed in port.c
    % Display ! {'$call', {self(), make_ref()}, {subscribe_input}},
    Display ! {self(), make_ref(), {subscribe_input, all}},

    loop().

disp(Display, Num) ->
    Bin = integer_to_binary(Num),
    Scene = [
        {text, 10, 20, default16px, Num, 16#FFFFFF, <<"Test ", Bin/binary>>}
    ],
    % TODO: let's switch back to the following as soon as tuple format is changed in port.c
    % Display ! {'$call', {self(), make_ref()}, {update, Scene}}.
    Display ! {self(), make_ref(), {update, Scene}}.

loop() ->
    receive
        Any -> erlang:display(Any)
    end,
    loop().
