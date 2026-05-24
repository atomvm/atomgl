%% SPDX-License-Identifier: Apache-2.0
%% SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com>

-module(atomgl_epaper_panels).

-export([
    panel/2
]).

-import(atomgl_epaper_descriptor, [
    acep7_panel/2,
    jd79656_panel/2,
    ssd16xx_panel/2,
    uc8151_panel/2,
    uc8276_panel/2
]).

-import(atomgl_epaper_program, [
    program/1,
    cmd/1,
    cmd/2,
    cmd_delay/3,
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

-define(LUT_FULL, 0).
-define(LUT_PARTIAL, 1).
-define(LUT_4GRAY, 2).
-define(LUT_FAST, 3).

panel("waveshare,epaper-2in9", Opts) ->
    panel("waveshare,epd2in9", Opts);
panel("waveshare,epd2in9", Opts) ->
    uc8151_panel("Waveshare epd2in9 2.9\" e-paper UC8151 v1", maps:merge(#{
        native_width => 128,
        native_height => 296,
        refresh_modes => [full, partial],
        default_refresh => full,
        init => uc8151_init(296, 16#03, ?LUT_FULL),
        full => uc8151_refresh(128, 296, ?LUT_FULL),
        partial => uc8151_refresh(128, 296, ?LUT_PARTIAL),
        sleep => cmd(16#10, <<16#01>>),
        lut_full => uc8151_2in9_full_lut(),
        lut_partial => uc8151_2in9_partial_lut()
    }, Opts));
panel("waveshare,epd2in9_V2", Opts) ->
    ssd16xx_panel("Waveshare epd2in9_V2 2.9\" e-paper SSD1680 v1", maps:merge(#{
        native_width => 128,
        native_height => 296,
        palette_size => 4,
        refresh_modes => [full, fast, partial, '4gray'],
        default_refresh => full,
        init => epd2in9_v2_init(none),
        full => epd2in9_v2_refresh(?LUT_FULL, 16#C7),
        fast => epd2in9_v2_refresh(?LUT_FAST, 16#C7),
        partial => epd2in9_v2_partial_refresh(),
        '4gray' => epd2in9_v2_4gray_refresh(),
        sleep => cmd(16#10, <<16#01>>),
        lut_full => epd2in9_v2_default_lut(),
        lut_fast => epd2in9_v2_fast_lut(),
        lut_partial => epd2in9_v2_partial_lut(),
        lut_4gray => epd2in9_v2_4gray_lut()
    }, Opts));
panel("waveshare,epd2in9_V2-fast", Opts) ->
    panel("waveshare,epd2in9_V2", maps:merge(#{default_refresh => fast}, Opts));
panel("waveshare,epd2in9_V2-partial", Opts) ->
    panel("waveshare,epd2in9_V2", maps:merge(#{default_refresh => partial}, Opts));
panel("waveshare,epd2in9_V2-4gray", Opts) ->
    panel("waveshare,epd2in9_V2", maps:merge(#{default_refresh => '4gray'}, Opts));
panel("dke,depg0290bns800f6", Opts) ->
    ssd16xx_panel("DKE DEPG0290BNS800F6 2.9\" e-paper SSD1680 v1", maps:merge(#{
        native_width => 128,
        native_height => 296,
        refresh_modes => [full],
        default_refresh => full,
        init => ssd1680_2in9_init(),
        full => ssd1680_2in9_refresh(),
        sleep => cmd(16#10, <<16#01>>)
    }, Opts));
panel("waveshare,epaper-2in13", Opts) ->
    panel("waveshare,epd2in13_V4", Opts);
panel("waveshare,epd2in13", Opts) ->
    uc8151_panel("Waveshare epd2in13 2.13\" e-paper UC8151 v1", maps:merge(#{
        native_width => 122,
        native_height => 250,
        refresh_modes => [full, partial],
        default_refresh => full,
        init => uc8151_init(250, 16#63, ?LUT_FULL),
        full => uc8151_refresh(122, 250, ?LUT_FULL),
        partial => uc8151_refresh(122, 250, ?LUT_PARTIAL),
        sleep => cmd(16#10, <<16#01>>),
        lut_full => uc8151_2in13_full_lut(),
        lut_partial => uc8151_2in13_partial_lut()
    }, Opts));
panel("waveshare,epd2in13_V4", Opts) ->
    ssd16xx_panel("Waveshare epd2in13_V4 2.13\" e-paper SSD1680 v1", maps:merge(#{
        native_width => 122,
        native_height => 250,
        refresh_modes => [full, fast, partial],
        default_refresh => full,
        init => epd2in13_v4_init(),
        full => epd2in13_v4_refresh(16#F7),
        fast => epd2in13_v4_refresh(16#C7),
        partial => epd2in13_v4_partial_refresh(),
        sleep => cmd(16#10, <<16#01>>)
    }, Opts));
panel("waveshare,epd2in13_V4-fast", Opts) ->
    panel("waveshare,epd2in13_V4", maps:merge(#{default_refresh => fast}, Opts));
panel("waveshare,epd2in13_V4-partial", Opts) ->
    panel("waveshare,epd2in13_V4", maps:merge(#{default_refresh => partial}, Opts));
panel("waveshare,epd4in2_V2", Opts) ->
    uc8276_panel("Waveshare epd4in2_V2 4.2\" e-paper UC8276 v1", maps:merge(#{
        native_width => 400,
        native_height => 300,
        palette_size => 4,
        refresh_modes => [full, fast, partial, '4gray'],
        default_refresh => full,
        init => uc8276_init(),
        full => uc8276_refresh(uc8276_mono_prelude(), 16#F7),
        fast => uc8276_refresh(uc8276_fast_prelude(16#6E), 16#C7),
        partial => uc8276_partial_refresh(),
        '4gray' => uc8276_4gray_refresh(),
        sleep => cmd(16#10, <<16#01>>),
        lut_4gray => uc8276_4gray_lut()
    }, Opts));
panel("waveshare,epd4in2_V2-fast", Opts) ->
    panel("waveshare,epd4in2_V2", maps:merge(#{default_refresh => fast}, Opts));
panel("waveshare,epd4in2_V2-partial", Opts) ->
    panel("waveshare,epd4in2_V2", maps:merge(#{default_refresh => partial}, Opts));
panel("waveshare,epd4in2_V2-4gray", Opts) ->
    panel("waveshare,epd4in2_V2", maps:merge(#{default_refresh => '4gray'}, Opts));
panel("heltec,lcmen2r13efc1", Opts) ->
    jd79656_panel("Heltec LCMEN2R13EFC1 2.13\" e-paper JD79656 v1", maps:merge(#{
        native_width => 122,
        native_height => 250,
        refresh_modes => [full],
        default_refresh => full,
        init => heltec_lcmen2r13efc1_init(),
        full => heltec_lcmen2r13efc1_refresh(),
        sleep => program([cmd(16#02), wait_busy(1, 5000)])
    }, Opts));
panel("heltec,icmen2r13efc1", Opts) ->
    panel("heltec,lcmen2r13efc1", Opts);
panel("heltec,ht-vme213", Opts) ->
    panel("heltec,lcmen2r13efc1", Opts);
panel("waveshare,5in65-acep-7c", Opts) ->
    acep7_panel("Waveshare 5.65\" ACeP 7-color", maps:merge(#{
        native_width => 600,
        native_height => 448,
        spi_clock_hz => 1000000,
        busy_idle_level => 1,
        use_gpio_pullups => true,
        palette => acep7,
        init_seq => acep7c_init_seq(),
        init_wait_busy_between_cmds => false,
        frame_preamble_seq => acep7c_frame_preamble_seq(),
        refresh_has_data => false,
        refresh_data_byte => 0,
        post_power_off_busy_level => 0,
        periodic_refresh_interval => 5
    }, Opts));
panel("good-display/gdep073e01", Opts) ->
    acep7_panel("Good Display GDEP073E01 7.3\" 7-color", maps:merge(#{
        native_width => 800,
        native_height => 480,
        spi_clock_hz => 4000000,
        busy_idle_level => 1,
        use_gpio_pullups => true,
        palette => gdep073e01,
        init_seq => gdep073e01_init_seq(),
        init_wait_busy_between_cmds => true,
        refresh_has_data => true,
        refresh_data_byte => 0,
        post_power_off_busy_level => 1,
        periodic_refresh_interval => 0
    }, Opts));
panel(_, _Opts) ->
    error.

uc8151_init(NativeHeight, BorderWaveform, LutSlot) ->
    DriverOutput = NativeHeight - 1,
    Border = case BorderWaveform of
        none -> [];
        _ -> [cmd(16#3C, <<BorderWaveform>>)]
    end,
    program([
        reset(200, 2, 200),
        cmd(16#01, <<DriverOutput:16/little-unsigned-integer, 16#00>>),
        cmd(16#0C, <<16#D7, 16#D6, 16#9D>>),
        cmd(16#2C, <<16#A8>>),
        cmd(16#3A, <<16#1A>>),
        cmd(16#3B, <<16#08>>),
        Border,
        cmd(16#11, <<16#03>>),
        insert_lut(LutSlot)
    ]).

uc8151_refresh(NativeWidth, NativeHeight, LutSlot) ->
    program([
        insert_lut(LutSlot),
        uc8151_window(NativeWidth, NativeHeight),
        uc8151_cursor(),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        cmd(16#22, <<16#C4>>),
        cmd(16#20),
        cmd(16#FF),
        delay_ms(100),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

uc8151_window(NativeWidth, NativeHeight) ->
    XEnd = NativeWidth - 1,
    YEnd = NativeHeight - 1,
    [
        cmd(16#44, <<16#00, ((XEnd bsr 3) band 16#FF)>>),
        cmd(16#45, <<16#00, 16#00, YEnd:16/little-unsigned-integer>>)
    ].

uc8151_cursor() ->
    [
        cmd(16#4E, <<16#00>>),
        cmd(16#4F, <<16#00, 16#00>>)
    ].

uc8276_init() ->
    program([
        reset(100, 2, 100),
        wait_busy(0, 5000),
        cmd(16#12),
        wait_busy(0, 5000),
        uc8276_mono_prelude(),
        wait_busy(0, 5000)
    ]).

uc8276_mono_prelude() ->
    [
        cmd(16#21, <<16#40, 16#00>>),
        cmd(16#3C, <<16#05>>),
        cmd(16#11, <<16#03>>),
        uc8276_window(),
        uc8276_cursor()
    ].

uc8276_fast_prelude(Temperature) ->
    [
        cmd(16#21, <<16#40, 16#00>>),
        cmd(16#3C, <<16#05>>),
        cmd(16#1A, <<Temperature>>),
        cmd(16#22, <<16#91>>),
        cmd(16#20),
        wait_busy(0, 5000),
        cmd(16#11, <<16#03>>),
        uc8276_window(),
        uc8276_cursor()
    ].

uc8276_refresh(Prelude, UpdateMode) ->
    program([
        Prelude,
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        cmd(16#26),
        insert_prev_frame(0),
        cmd(16#22, <<UpdateMode>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

uc8276_partial_refresh() ->
    program([
        cmd(16#3C, <<16#80>>),
        cmd(16#21, <<16#00, 16#00>>),
        cmd(16#3C, <<16#80>>),
        uc8276_window(),
        uc8276_cursor(),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        cmd(16#22, <<16#FF>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

uc8276_4gray_refresh() ->
    program([
        reset(100, 2, 100),
        cmd(16#12),
        wait_busy(0, 5000),
        cmd(16#21, <<16#00, 16#00>>),
        cmd(16#3C, <<16#03>>),
        cmd(16#0C, <<16#8B, 16#9C, 16#A4, 16#0F>>),
        insert_lut(?LUT_4GRAY),
        cmd(16#11, <<16#03>>),
        uc8276_window(),
        uc8276_cursor(),
        cmd(16#24),
        insert_plane(0),
        cmd(16#26),
        insert_plane(1),
        cmd(16#22, <<16#CF>>),
        cmd(16#20),
        wait_busy(0, 5000)
    ]).

uc8276_window() ->
    [cmd(16#44, <<16#00, 16#31>>), cmd(16#45, <<16#00, 16#00, 16#2B, 16#01>>)].

uc8276_cursor() ->
    [cmd(16#4E, <<16#00>>), cmd(16#4F, <<16#00, 16#00>>)].

ssd1680_2in9_init() ->
    program([
        reset(10, 10, 100),
        wait_busy(0, 5000),
        cmd(16#12),
        wait_busy(0, 5000),
        label_delay(0, 20),
        cmd(16#3C, <<16#05>>),
        cmd(16#2C, <<16#36>>),
        cmd(16#03, <<16#17>>),
        cmd(16#04, <<16#41, 16#00, 16#32>>),
        cmd(16#11, <<16#03>>),
        cmd(16#44, <<16#01, 16#10>>),
        cmd(16#45, <<16#00, 16#00, 16#27, 16#01>>),
        set_ram_cursor(1),
        cmd(16#01, <<16#27, 16#01, 16#00>>)
    ]).

ssd1680_2in9_refresh() ->
    program([
        set_ram_cursor(1),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        delay_ms(2),
        set_ram_cursor(1),
        cmd(16#26),
        insert_plane(1),
        cmd(16#22, <<16#F4>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

epd2in13_v4_init() ->
    program([
        reset(20, 2, 20),
        wait_busy(0, 5000),
        cmd(16#12),
        wait_busy(0, 5000),
        cmd(16#01, <<16#F9, 16#00, 16#00>>),
        cmd(16#11, <<16#03>>),
        epd2in13_v4_window(),
        epd2in13_v4_cursor(),
        cmd(16#3C, <<16#05>>),
        cmd(16#21, <<16#00, 16#80>>),
        cmd(16#18, <<16#80>>),
        wait_busy(0, 5000)
    ]).

epd2in13_v4_refresh(UpdateMode) ->
    program([
        epd2in13_v4_cursor(),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        epd2in13_v4_cursor(),
        cmd(16#26),
        insert_prev_frame(0),
        cmd(16#22, <<UpdateMode>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

epd2in13_v4_partial_refresh() ->
    program([
        reset(0, 1, 0),
        cmd(16#3C, <<16#80>>),
        cmd(16#01, <<16#F9, 16#00, 16#00>>),
        cmd(16#11, <<16#03>>),
        epd2in13_v4_window(),
        epd2in13_v4_cursor(),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        cmd(16#22, <<16#FF>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

epd2in13_v4_window() ->
    [cmd(16#44, <<16#00, 16#0F>>), cmd(16#45, <<16#00, 16#00, 16#F9, 16#00>>)].

epd2in13_v4_cursor() ->
    [cmd(16#4E, <<16#00>>), cmd(16#4F, <<16#00, 16#00>>)].

%% SPDX-SnippetBegin
%% SPDX-SnippetCopyrightText: Copyright (c) 2019 Heltec Automation
%% SPDX-License-Identifier: MIT
heltec_lcmen2r13efc1_init() ->
    program([
        reset(100, 100, 100),
        wait_busy(1, 5000),
        cmd(16#12),
        wait_busy(1, 5000),
        cmd(16#4D, <<16#55, 16#00, 16#00>>),
        cmd(16#A9, <<16#25, 16#00, 16#00>>),
        cmd(16#F3, <<16#0A, 16#00, 16#00>>),
        cmd(16#44, <<16#01, 16#0F>>),
        cmd(16#45, <<16#F9, 16#00, 16#00, 16#00>>),
        cmd(16#3C, <<16#01>>),
        cmd(16#18, <<16#80>>),
        cmd(16#4E, <<16#01>>),
        cmd(16#4F, <<16#F9, 16#00>>),
        wait_busy(1, 5000)
    ]).

heltec_lcmen2r13efc1_refresh() ->
    program([
        cmd(16#13),
        insert_plane(0),
        cmd(16#12),
        cmd(16#04),
        wait_busy(1, 5000),
        delay_ms(10),
        cmd(16#12),
        delay_ms(10),
        wait_busy(1, 5000),
        cmd(16#02),
        wait_busy(1, 5000)
    ]).
%% SPDX-SnippetEnd

acep7c_init_seq() ->
    program([
        cmd(16#00, <<16#EF, 16#08>>),
        cmd(16#01, <<16#37, 16#00, 16#23, 16#23>>),
        cmd(16#03, <<16#00>>),
        cmd(16#06, <<16#C7, 16#C7, 16#1D>>),
        cmd(16#30, <<16#3C>>),
        cmd(16#40, <<16#00>>),
        cmd(16#50, <<16#3F>>),
        cmd(16#60, <<16#22>>),
        cmd(16#61, <<16#02, 16#58, 16#01, 16#C0>>),
        cmd(16#E3, <<16#AA>>),
        cmd_delay(16#82, <<16#80>>, 100),
        cmd(16#50, <<16#37>>)
    ]).

acep7c_frame_preamble_seq() ->
    program([
        cmd(16#61, <<16#02, 16#58, 16#01, 16#C0>>)
    ]).

gdep073e01_init_seq() ->
    program([
        cmd(16#AA, <<16#49, 16#55, 16#20, 16#08, 16#09, 16#18>>),
        cmd(16#01, <<16#3F>>),
        cmd(16#00, <<16#5F, 16#69>>),
        cmd(16#03, <<16#00, 16#54, 16#00, 16#44>>),
        cmd(16#05, <<16#40, 16#1F, 16#1F, 16#2C>>),
        cmd(16#06, <<16#6F, 16#1F, 16#17, 16#49>>),
        cmd(16#08, <<16#6F, 16#1F, 16#1F, 16#22>>),
        cmd(16#30, <<16#00>>),
        cmd(16#50, <<16#3F>>),
        cmd(16#60, <<16#02, 16#00>>),
        cmd(16#61, <<16#03, 16#20, 16#01, 16#E0>>),
        cmd(16#84, <<16#01>>),
        cmd(16#E3, <<16#2F>>),
        cmd(16#04)
    ]).

epd2in9_v2_init(BorderWaveform) ->
    epd2in9_v2_init(BorderWaveform, 0).

epd2in9_v2_init(BorderWaveform, RamXOffset) ->
    RamXEnd = RamXOffset + 15,
    Base = [
        reset(10, 10, 100),
        wait_busy(0, 5000),
        cmd(16#12),
        wait_busy(0, 5000),
        label_delay(0, 20),
        cmd(16#01, <<16#27, 16#01, 16#00>>),
        cmd(16#11, <<16#03>>),
        cmd(16#44, <<RamXOffset, RamXEnd>>),
        cmd(16#45, <<16#00, 16#00, 16#27, 16#01>>),
        cmd(16#4E, <<RamXOffset>>),
        cmd(16#4F, <<16#00, 16#00>>),
        cmd(16#21, <<16#00, 16#80>>)
    ],
    program(Base ++ border_waveform(BorderWaveform)).

epd2in9_v2_refresh(LutSlot, UpdateMode) ->
    program([
        insert_lut(LutSlot),
        set_ram_cursor(),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        delay_ms(2),
        set_ram_cursor(),
        cmd(16#26),
        insert_prev_frame(0),
        cmd(16#22, <<UpdateMode>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

epd2in9_v2_partial_refresh() ->
    program([
        epd2in9_v2_partial_prelude(),
        cmd(16#11, <<16#03>>),
        cmd(16#44, <<16#00, 16#0F>>),
        cmd(16#45, <<16#00, 16#00, 16#27, 16#01>>),
        set_ram_cursor(),
        cmd(16#26),
        insert_prev_frame(0),
        delay_ms(2),
        set_ram_cursor(),
        cmd(16#24),
        capture_frame(),
        insert_plane(0),
        cmd(16#22, <<16#0F>>),
        cmd(16#20),
        wait_busy(0, 5000),
        mark_prev_valid()
    ]).

epd2in9_v2_partial_prelude() ->
    [
        reset(0, 1, 2),
        insert_lut(?LUT_PARTIAL),
        cmd(16#37, <<16#00, 16#00, 16#00, 16#00, 16#00, 16#40, 16#00, 16#00, 16#00, 16#00>>),
        cmd(16#3C, <<16#80>>),
        cmd(16#22, <<16#C0>>),
        cmd(16#20),
        wait_busy(0, 5000)
    ].

epd2in9_v2_4gray_refresh() ->
    program([
        insert_lut(?LUT_4GRAY),
        set_ram_cursor(),
        cmd(16#24),
        insert_plane(0),
        delay_ms(2),
        set_ram_cursor(),
        cmd(16#26),
        insert_plane(1),
        cmd(16#22, <<16#C7>>),
        cmd(16#20),
        wait_busy(0, 5000)
    ]).

border_waveform(none) ->
    [];
border_waveform(Waveform) ->
    [cmd(16#3C, <<Waveform>>)].

set_ram_cursor() ->
    set_ram_cursor(0).

set_ram_cursor(RamXOffset) ->
    [cmd(16#4E, <<RamXOffset>>), cmd(16#4F, <<16#00, 16#00>>)].

%% SPDX-SnippetBegin
%% SPDX-SnippetCopyrightText: Waveshare team
%% SPDX-License-Identifier: MIT
uc8151_2in9_full_lut() ->
    <<
        16#50, 16#AA, 16#55, 16#AA, 16#11, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#FF, 16#FF, 16#1F, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00
    >>.

uc8151_2in9_partial_lut() ->
    <<
        16#10, 16#18, 16#18, 16#08, 16#18, 16#18,
        16#08, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#13, 16#14, 16#44, 16#12,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00
    >>.

uc8151_2in13_full_lut() ->
    <<
        16#22, 16#55, 16#AA, 16#55, 16#AA, 16#55, 16#AA, 16#11,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#1E, 16#1E, 16#1E, 16#1E, 16#1E, 16#1E, 16#1E, 16#1E,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#00
    >>.

uc8151_2in13_partial_lut() ->
    <<
        16#18, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#0F, 16#01, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00
    >>.

uc8276_4gray_lut() ->
    <<
        16#01, 16#0A, 16#1B, 16#0F, 16#03, 16#01, 16#01,
        16#05, 16#0A, 16#01, 16#0A, 16#01, 16#01, 16#01,
        16#05, 16#08, 16#03, 16#02, 16#04, 16#01, 16#01,
        16#01, 16#04, 16#04, 16#02, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#0A, 16#1B, 16#0F, 16#03, 16#01, 16#01,
        16#05, 16#4A, 16#01, 16#8A, 16#01, 16#01, 16#01,
        16#05, 16#48, 16#03, 16#82, 16#84, 16#01, 16#01,
        16#01, 16#84, 16#84, 16#82, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#0A, 16#1B, 16#8F, 16#03, 16#01, 16#01,
        16#05, 16#4A, 16#01, 16#8A, 16#01, 16#01, 16#01,
        16#05, 16#48, 16#83, 16#82, 16#04, 16#01, 16#01,
        16#01, 16#04, 16#04, 16#02, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#8A, 16#1B, 16#8F, 16#03, 16#01, 16#01,
        16#05, 16#4A, 16#01, 16#8A, 16#01, 16#01, 16#01,
        16#05, 16#48, 16#83, 16#02, 16#04, 16#01, 16#01,
        16#01, 16#04, 16#04, 16#02, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#8A, 16#9B, 16#8F, 16#03, 16#01, 16#01,
        16#05, 16#4A, 16#01, 16#8A, 16#01, 16#01, 16#01,
        16#05, 16#48, 16#03, 16#42, 16#04, 16#01, 16#01,
        16#01, 16#04, 16#04, 16#42, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#01, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#02, 16#00, 16#00, 16#07, 16#17, 16#41, 16#A8,
        16#32, 16#30
    >>.

epd2in9_v2_partial_lut() ->
    <<
        16#00, 16#40, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#80, 16#80, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#40, 16#40, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#80, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#0A, 16#00, 16#00, 16#00, 16#00, 16#00, 16#01, 16#01, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#01, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#22, 16#22, 16#22, 16#22, 16#22, 16#22, 16#00, 16#00, 16#00, 16#22, 16#17, 16#41,
        16#B0, 16#32, 16#36
    >>.

epd2in9_v2_default_lut() ->
    <<
        16#80, 16#66, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#40, 16#00, 16#00, 16#00,
        16#10, 16#66, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#20, 16#00, 16#00, 16#00,
        16#80, 16#66, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#40, 16#00, 16#00, 16#00,
        16#10, 16#66, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#20, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#14, 16#08, 16#00, 16#00, 16#00, 16#00, 16#02, 16#0A, 16#0A, 16#00, 16#0A, 16#0A,
        16#00, 16#01, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#14, 16#08, 16#00, 16#01,
        16#00, 16#00, 16#01, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#01, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#44, 16#44, 16#44, 16#44, 16#44, 16#44, 16#00, 16#00, 16#00, 16#22, 16#17, 16#41,
        16#00, 16#32, 16#36
    >>.

epd2in9_v2_4gray_lut() ->
    <<
        16#00, 16#60, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#20, 16#60, 16#10, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#28, 16#60, 16#14, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#2A, 16#60, 16#15, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#90, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#02, 16#00, 16#05, 16#14, 16#00, 16#00, 16#1E, 16#1E, 16#00, 16#00, 16#00,
        16#00, 16#01, 16#00, 16#02, 16#00, 16#05, 16#14, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#24, 16#22, 16#22, 16#22, 16#23, 16#32, 16#00, 16#00, 16#00, 16#22, 16#17, 16#41,
        16#AE, 16#32, 16#28
    >>.

epd2in9_v2_fast_lut() ->
    <<
        16#90, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#60, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#90, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#60, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#19, 16#19, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00, 16#00,
        16#24, 16#42, 16#22, 16#22, 16#23, 16#32, 16#00, 16#00, 16#00, 16#22, 16#17, 16#41,
        16#AE, 16#32, 16#38
    >>.
%% SPDX-SnippetEnd
