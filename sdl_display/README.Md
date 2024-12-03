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
