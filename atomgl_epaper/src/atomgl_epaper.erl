%% SPDX-License-Identifier: Apache-2.0
%% SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com>

-module(atomgl_epaper).

-export([
    panel/1,
    panel/2,
    validate_descriptor/1
]).

panel(Compatible) when is_binary(Compatible) ->
    panel(binary_to_list(Compatible), #{});
panel(Compatible) ->
    panel(Compatible, #{}).

panel(Compatible, Opts) when is_binary(Compatible) ->
    panel(binary_to_list(Compatible), Opts);
panel(Compatible, Opts) ->
    atomgl_epaper_panels:panel(Compatible, Opts).

validate_descriptor(Desc) ->
    atomgl_epaper_descriptor:validate_descriptor(Desc).
