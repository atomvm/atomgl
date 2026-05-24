<!---
  Copyright 2026 Peter M <petermm@gmail.com>

  SPDX-License-Identifier: Apache-2.0
-->

# atomgl_epaper

A small Erlang/OTP library that provides ready-made panel descriptors and a
bytecode builder for e-paper displays driven by [AtomGL](../README.Md).

The main public entry point is `atomgl_epaper`, whose `panel/1` and
`panel/2` functions return a validated descriptor (`{ok, Descriptor}`) that
can be passed straight to the AtomGL e-paper display driver.

For custom out-of-tree panels, `atomgl_epaper_descriptor` and
`atomgl_epaper_program` can be used directly from your own project. See
[`CUSTOM_PANEL.md`](CUSTOM_PANEL.md).

## Usage

```erlang
{ok, Descriptor} = atomgl_epaper:panel("waveshare,epd2in9_V2"),
%% Descriptor can now be used to open the AtomGL display port.
```

An optional options map may override defaults such as `orientation`,
`rotation`, `view_width`, `view_height`, `spi_clock_hz`, etc.:

```erlang
{ok, Descriptor} =
    atomgl_epaper:panel("waveshare,epd2in9_V2",
                        #{orientation => portrait}).
```

## Adding this library to your project

This library lives inside the [AtomGL](https://github.com/atomvm/atomgl)
mono-repo. Both `rebar3` and `mix` can pull just this subdirectory.

### rebar3

```erlang
%% rebar.config
{deps, [
    {atomgl_epaper,
        {git, "https://github.com/atomvm/atomgl.git", {branch, "main"}},
        [{subdir, "atomgl_epaper"}]}
]}.
```

### Elixir (Mix)

```elixir
# mix.exs
defp deps do
  [
    {:atomgl_epaper,
      github: "atomvm/atomgl",
      branch: "main",
      sparse: "atomgl_epaper"}
  ]
end
```

## Tests

```bash
cd atomgl_epaper
rebar3 eunit
```

## License

Apache-2.0. See [`LICENSE`](LICENSE).
