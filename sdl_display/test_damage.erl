%
% This file is part of AtomGL.
%
% Copyright 2026 Davide Bettio <davide@uninstall.it>
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

%% Pushes a sequence of display lists that exercises the partial repaint
%% logic: items changing in place, appearing, disappearing, moving, changing
%% z-order, identical frames, off screen and invalid items, many changes at
%% once, text in a ufont that stays while other items change or gets replaced.
%% Run with the SDL driver in verify
%% mode, headless if needed:
%%
%%   AVM_SDL_DISPLAY_VERIFY_DAMAGE=1 SDL_VIDEODRIVER=dummy \
%%       AtomVM test_damage.beam /path/to/atomvmlib.avm
%%
%% Every frame is then compared with a full render; a mismatch prints a "FAILED"
%% line, fails the update call and makes the program exit with an error.
-module(test_damage).
-export([start/0]).

-define(WIDTH, 160).
-define(HEIGHT, 120).

start() ->
    Display = erlang:open_port({spawn, "display"}, [{width, ?WIDTH}, {height, ?HEIGHT}]),
    % 24x16 pixels: a reference counted binary, shared by every frame using it.
    Big = image(24, 16, fun(X, Y) -> [X * 10, Y * 15, 128, alpha(X, Y)] end),
    % 2x2 pixels: a heap binary, copied into every message.
    Tiny = image(2, 2, fun(_X, _Y) -> [255, 0, 0, 255] end),
    Tiny2 = image(2, 2, fun(_X, _Y) -> [255, 0, 0, 255] end),
    Scenes = scenes(Big, Tiny, Tiny2),
    Failed = run(Display, Scenes, 1, 0),
    Frames = length([Scene || Scene <- Scenes, is_list(Scene)]),
    erlang:display({test_damage, done, Frames, failed, Failed}),
    case Failed of
        0 -> ok;
        _ -> {error, {failed, Failed}}
    end.

alpha(X, Y) when (X + Y) rem 3 =:= 0 -> 0;
alpha(X, Y) when (X + Y) rem 3 =:= 1 -> 128;
alpha(_X, _Y) -> 255.

image(W, H, Fun) ->
    Rows = [[Fun(X, Y) || X <- lists:seq(0, W - 1)] || Y <- lists:seq(0, H - 1)],
    {rgba8888, W, H, list_to_binary(Rows)}.

bg() ->
    {rect, 0, 0, ?WIDTH, ?HEIGHT, 16#808080}.

%% A ufont with 4x5 pixel glyphs for $A, $B, ... given as rows of "X" (ink) and "." (blank).
ufont(Glyphs) ->
    Count = length(Glyphs),
    Header = <<1:32/little, 0, 6:16/little, 5:16/little, 0:16/little>>,
    GlyphRecords = [
        <<4:16/little, 5:16/little, 5:16/little, 0:16/little, 5:16/little, 0:32/little,
            (I * 10):32/little>>
     || I <- lists:seq(0, Count - 1)
    ],
    Intervals = <<$A:32/little, ($A + Count - 1):32/little, 0:32/little>>,
    Bitmap = [glyph_bitmap(Rows) || Rows <- Glyphs],
    Body = list_to_binary([
        chunk("uFH0", Header),
        chunk("uFP0", GlyphRecords),
        chunk("uFI0", Intervals),
        chunk("uFB0", Bitmap)
    ]),
    Size = 12 + byte_size(Body),
    <<"FORM", Size:32/big, "uFL0", Body/binary>>.

chunk(Name, Data) ->
    Bin = list_to_binary([Data]),
    Pad = (4 - byte_size(Bin) rem 4) rem 4,
    [Name, <<(byte_size(Bin)):32/big>>, Bin, lists:duplicate(Pad, 0)].

%% Two pixels per byte, the left one in the low nibble.
glyph_bitmap(Rows) ->
    [[ink(P0) bor (ink(P1) bsl 4), ink(P2) bor (ink(P3) bsl 4)] || [P0, P1, P2, P3] <- Rows].

ink($X) -> 15;
ink($.) -> 0.

grid(Color, Columns) ->
    [{rect, X * 12, Y * 25, 6, 6, Color} || X <- lists:seq(0, Columns - 1), Y <- lists:seq(0, 3)].

scenes(Big, Tiny, Tiny2) ->
    Text = {text, 10, 10, default16px, 16#000000, transparent, <<"Hello">>},
    FontA = ufont([
        ["..XX", ".X.X", "XXXX", "X..X", "X..X"],
        ["XXX.", "X..X", "XXX.", "X..X", "XXX."]
    ]),
    FontB = ufont([
        ["XXXX", "XXXX", "XXXX", "XXXX", "XXXX"],
        ["X.X.", ".X.X", "X.X.", ".X.X", "X.X."]
    ]),
    UText = {text, 20, 60, tiny, 16#FFFF00, transparent, <<"ABBA">>},
    [
        [bg()],
        % identical frame
        [bg()],
        [Text, bg()],
        % identical frame again
        [Text, bg()],
        % text changed in place
        [{text, 10, 10, default16px, 16#000000, transparent, <<"Hellp">>}, bg()],
        % colors and background changed
        [{text, 10, 10, default16px, 16#FF0000, 16#FFFFFF, <<"Hellp">>}, bg()],
        % image appears
        [Text, {image, 60, 40, transparent, Big}, bg()],
        % image moves
        [Text, {image, 70, 50, transparent, Big}, bg()],
        % background behind translucent pixels becomes visible
        [Text, {image, 70, 50, 16#00FF00, Big}, bg()],
        % tiny heap binary image
        [Text, {image, 70, 50, 16#00FF00, Big}, {image, 5, 100, transparent, Tiny}, bg()],
        % same pixels through another binary: repainted, pixels are never compared
        [Text, {image, 70, 50, 16#00FF00, Big}, {image, 5, 100, transparent, Tiny2}, bg()],
        % z-order change of non overlapping items
        [{image, 70, 50, 16#00FF00, Big}, Text, {image, 5, 100, transparent, Tiny2}, bg()],
        % overlapping items swapping z-order
        [{rect, 20, 20, 30, 30, 16#FF0000}, {rect, 35, 35, 30, 30, 16#0000FF}, bg()],
        [{rect, 35, 35, 30, 30, 16#0000FF}, {rect, 20, 20, 30, 30, 16#FF0000}, bg()],
        % many changes at once, more than the damage rectangles available
        grid(16#00FFFF, 12) ++ [bg()],
        grid(16#FFFF00, 12) ++ [bg()],
        grid(16#FFFF00, 6) ++ [bg()],
        % partially and fully off screen items
        [
            {rect, -10, -10, 30, 30, 16#FF00FF},
            {rect, 150, 110, 30, 30, 16#FF00FF},
            {rect, 300, 300, 10, 10, 16#FFFFFF},
            {text, -20, 50, default16px, 16#000000, transparent, <<"Edge">>},
            {text, 140, 100, default16px, 16#000000, 16#FFFFFF, <<"Corner">>},
            bg()
        ],
        % scaled and cropped images, the second one with a clamped bounding box
        [{scaled_cropped_image, 20, 20, 24, 16, transparent, 4, 2, 2, 2, [], Big}, bg()],
        [{scaled_cropped_image, 20, 20, 100, 100, 16#0000FF, 4, 2, 3, 3, [], Big}, bg()],
        [{scaled_cropped_image, 22, 20, 100, 100, 16#0000FF, 4, 2, 3, 3, [], Big}, bg()],
        % shapes appear, change color, move and overlap text
        [{circle, 40, 60, 15, 16#FF0000}, Text, bg()],
        [{circle, 40, 60, 15, 16#00FF00}, Text, bg()],
        [{circle, 45, 62, 15, 16#00FF00}, Text, bg()],
        [{rounded_rect, 60, 20, 60, 30, 8, 16#3060C0}, {line, 10, 100, 150, 110, 3, 16#FFCC00}, Text, bg()],
        [{rounded_rect, 60, 20, 60, 30, 4, 16#3060C0}, {line, 10, 100, 150, 112, 3, 16#FFCC00}, Text, bg()],
        [{ellipse, 100, 70, 30, 12, 16#C04080}, {arc, 120, 60, 25, 6, -90, 180, 16#00C0C0}, bg()],
        [{ellipse, 100, 70, 30, 12, 16#C04080}, {arc, 120, 60, 25, 6, -90, 90, 16#00C0C0}, bg()],
        % same polygon twice, then one vertex moves
        [{polygon, [{10, 110}, {50, 70}, {90, 110}, {50, 95}], 16#E06020}, bg()],
        [{polygon, [{10, 110}, {50, 70}, {90, 110}, {50, 95}], 16#E06020}, bg()],
        [{polygon, [{10, 110}, {50, 70}, {90, 110}, {50, 100}], 16#E06020}, bg()],
        % a ring over text: the text shows through the hole
        [{arc, 80, 60, 40, 8, 0, 360, 16#FFFFFF}, {text, 50, 52, default16px, 16#FFFF00, transparent, <<"Ring">>}, bg()],
        % flips of a scaled image
        [{scaled_cropped_image, 20, 20, 48, 32, transparent, 0, 0, 2, 2, [], Big}, bg()],
        [{scaled_cropped_image, 20, 20, 48, 32, transparent, 0, 0, 2, 2, [flip_x], Big}, bg()],
        [{scaled_cropped_image, 20, 20, 48, 32, transparent, 0, 0, 2, 2, [flip_y], Big}, bg()],
        [{scaled_cropped_image, 20, 20, 48, 32, 16#004000, 0, 0, 2, 2, [flip_x, flip_y], Big}, bg()],
        % text in a ufont: the same text again, then another text and other colors
        {register_font, tiny, FontA},
        [UText, bg()],
        [UText, bg()],
        [{text, 20, 60, tiny, 16#FFFF00, transparent, <<"BAAB">>}, bg()],
        [{text, 20, 60, tiny, 16#00FFFF, 16#400040, <<"BAAB">>}, bg()],
        [UText, bg()],
        % the same text while something else changes: neither repainted nor rasterized
        [UText, {rect, 100, 10, 20, 20, 16#FF0000}, bg()],
        [UText, {rect, 100, 10, 20, 20, 16#00FF00}, bg()],
        % the same text over a rect that changes: repainted, hence rasterized again
        [UText, {rect, 10, 55, 60, 20, 16#202020}, bg()],
        [UText, {rect, 10, 55, 60, 20, 16#404040}, bg()],
        % the same text after the font was replaced under the same handle, then removed
        {deregister_font, tiny},
        {register_font, tiny, FontB},
        [UText, bg()],
        {deregister_font, tiny},
        [UText, bg()],
        % invalid items are reported and ignored
        [
            {rect, 1, 2},
            {text, 10, 10, default16px, 16#000000, transparent, <<"Hi">>},
            {image, 0, 0, transparent, {rgba8888, 100, 100, <<1, 2, 3>>}},
            {scaled_cropped_image, 20, 20, 24, 16, transparent, 4, 2, 0, 2, [], Big},
            bg()
        ],
        % empty list, then back
        [],
        [bg()],
        [Text, bg()],
        % no background: uncovered pixels are black
        [Text],
        [{text, 10, 40, default16px, 16#FFFFFF, transparent, <<"Dark">>}],
        [bg()]
    ].

run(_Display, [], _N, Failed) ->
    Failed;
run(Display, [Scene | Rest], N, Failed) when is_list(Scene) ->
    case call(Display, {update, Scene}) of
        ok ->
            run(Display, Rest, N + 1, Failed);
        Other ->
            erlang:display({update_failed, N, Other}),
            run(Display, Rest, N + 1, Failed + 1)
    end;
% a font command between two frames
run(Display, [Req | Rest], N, Failed) ->
    ok = call(Display, Req),
    run(Display, Rest, N, Failed).

call(Port, Req) ->
    Ref = make_ref(),
    Port ! {'$call', {self(), Ref}, Req},
    receive
        {Ref, Reply} -> Reply
    after 5000 ->
        {error, timeout}
    end.
