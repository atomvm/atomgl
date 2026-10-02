#
# This file is part of AtomGL.
#
# Copyright 2026 Tom Hoenderdos <tomhoenderdos@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required (VERSION 3.13)
project (atomgl_bench C)

set(CMAKE_C_STANDARD 99)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if (NOT LIBATOMVM_INCLUDE_PATH)
    message(FATAL_ERROR "Set LIBATOMVM_INCLUDE_PATH to AtomVM's src/libAtomVM (headers only, nothing is linked)")
endif()

add_executable(bench bench.c ../../dcs_lcd_draw.c ../../font_data.c ../../shape.c)
target_include_directories(bench PRIVATE ../.. ../stubs ${LIBATOMVM_INCLUDE_PATH})
target_compile_definitions(bench PRIVATE HAVE_ATOMIC)
target_compile_options(bench PRIVATE -Wall -Wextra -Wno-unused-parameter)

add_executable(test_flip test_flip.c)
target_include_directories(test_flip PRIVATE ../.. ../stubs ${LIBATOMVM_INCLUDE_PATH})
target_compile_definitions(test_flip PRIVATE HAVE_ATOMIC)
target_compile_options(test_flip PRIVATE -Wall -Wextra -Wno-unused-parameter)
