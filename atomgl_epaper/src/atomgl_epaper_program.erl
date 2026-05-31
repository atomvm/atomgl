%% SPDX-License-Identifier: Apache-2.0
%% SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com>

-module(atomgl_epaper_program).

-export([
    program/1,
    init_seq/1,
    cmd/1,
    cmd/2,
    cmd_delay/3,
    init_cmd/1,
    init_cmd/2,
    init_cmd_delay/3,
    wait_busy/2,
    reset/3,
    insert_plane/1,
    insert_lut/1,
    insert_prev_frame/1,
    capture_frame/0,
    mark_prev_valid/0,
    label_delay/2,
    delay_ms/1
]).

-define(DELAY, 16#80).
-define(META, 16#40).
-define(LEN_MASK, 16#3F).
-define(INIT_LEN_MASK, 16#7F).

-define(OP_WAIT_BUSY, 16#01).
-define(OP_RESET, 16#02).
-define(OP_INSERT_PLANE, 16#03).
-define(OP_INSERT_LUT, 16#04).
-define(OP_INSERT_PREV_FRAME, 16#05).
-define(OP_CAPTURE_FRAME, 16#06).
-define(OP_MARK_PREV_VALID, 16#07).
-define(OP_LABEL, 16#10).

program(Parts) ->
    iolist_to_binary(Parts).

init_seq(Parts) ->
    iolist_to_binary(Parts).

cmd(Cmd) ->
    cmd(Cmd, <<>>).

cmd(Cmd, Data) when is_list(Data) ->
    cmd(Cmd, list_to_binary(Data));
cmd(Cmd, Data) when byte_size(Data) =< ?LEN_MASK ->
    <<Cmd, (byte_size(Data)), Data/binary>>.

cmd_delay(Cmd, Data, DelayMs) when byte_size(Data) =< ?LEN_MASK ->
    <<Cmd, (?DELAY bor byte_size(Data)), Data/binary, DelayMs>>.

init_cmd(Cmd) ->
    init_cmd(Cmd, <<>>).

init_cmd(Cmd, Data) when is_list(Data) ->
    init_cmd(Cmd, list_to_binary(Data));
init_cmd(Cmd, Data) when byte_size(Data) =< ?INIT_LEN_MASK ->
    <<Cmd, (byte_size(Data)), Data/binary>>.

init_cmd_delay(Cmd, Data, DelayMs) when is_list(Data) ->
    init_cmd_delay(Cmd, list_to_binary(Data), DelayMs);
init_cmd_delay(Cmd, Data, DelayMs) when byte_size(Data) =< ?INIT_LEN_MASK ->
    <<Cmd, (?DELAY bor byte_size(Data)), Data/binary, DelayMs>>.

wait_busy(Level, TimeoutMs) ->
    meta(?OP_WAIT_BUSY, <<Level, TimeoutMs:16/little-unsigned-integer>>).

reset(HighMs, LowMs, SettleMs) ->
    meta(?OP_RESET, <<HighMs, LowMs, SettleMs>>).

insert_plane(PlaneId) ->
    meta(?OP_INSERT_PLANE, <<PlaneId>>).

insert_lut(SlotId) ->
    meta(?OP_INSERT_LUT, <<SlotId>>).

insert_prev_frame(PlaneId) ->
    meta(?OP_INSERT_PREV_FRAME, <<PlaneId>>).

capture_frame() ->
    meta(?OP_CAPTURE_FRAME, <<>>).

mark_prev_valid() ->
    meta(?OP_MARK_PREV_VALID, <<>>).

label_delay(LabelId, DelayMs) ->
    meta_delay(?OP_LABEL, <<LabelId>>, DelayMs).

delay_ms(DelayMs) ->
    label_delay(0, DelayMs).

meta(Opcode, Operands) when byte_size(Operands) =< ?LEN_MASK ->
    <<Opcode, (?META bor byte_size(Operands)), Operands/binary>>.

meta_delay(Opcode, Operands, DelayMs) when byte_size(Operands) =< ?LEN_MASK ->
    <<Opcode, (?META bor ?DELAY bor byte_size(Operands)), Operands/binary, DelayMs>>.
