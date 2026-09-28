<!---
  Copyright 2020-2024 Davide Bettio <davide@uninstall.it>

  SPDX-License-Identifier: Apache-2.0
-->

# SDL Display Driver

This display driver can be used for displaying graphics on a PC using SDL library.

## Build
In order to build it, just run `cmake` first and then `make`.

Beware:
- Make sure to set the path to AtomVM headers (`LIBATOMVM_INCLUDE_PATH`) before running cmake,
such as: `cmake -DLIBATOMVM_INCLUDE_PATH=/path-to/AtomVM/src/libAtomVM/ .`.
- Also make sure to build this plugin using same options as AtomVM, that means that
when `AVM_DISABLE_SMP` or `AVM_DISABLE_TASK_DRIVER` have been disabled in AtomVM build, also here
they must be disabled accordingly (their default is on, and in that case no further action is
required).

## Requirements

- zlib
- SDL (v1)

## Run

Once compiled, it must placed in the current working directory.

## Testing Partial Updates

The driver repaints only the areas that changed between two updates. Set
`AVM_SDL_DISPLAY_VERIFY_DAMAGE=1` to have every frame rendered again from scratch and compared with
the incrementally updated frame buffer; any mismatch is reported on stderr. `test_damage.erl` pushes
a sequence of display lists exercising the update logic and can run headless:

```bash
erlc test_damage.erl
AVM_SDL_DISPLAY_VERIFY_DAMAGE=1 SDL_VIDEODRIVER=dummy AtomVM test_damage.beam /path/to/atomvmlib.avm
```

Every `damage verify: frame N ok` line is a verified frame. A `FAILED` line is a bug: the `update`
call then returns `{error, verify_failed}` and `test_damage` exits with a non-zero status.
