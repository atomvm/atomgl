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
project (atomgl_items_tests C)

set(CMAKE_C_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

if (NOT LIBATOMVM_INCLUDE_PATH)
    message(FATAL_ERROR "Set LIBATOMVM_INCLUDE_PATH to AtomVM's src/libAtomVM (headers only, nothing is linked)")
endif()

option(ITEMS_SANITIZE "Build with ASan and UBSan, aborting on the first report" ON)

add_executable(test_items test_items.c atomvm_stubs.c alloc_counter.c
    ../../display_items.c ../../dcs_lcd_draw.c ../../font_data.c)
set_source_files_properties(../../display_items.c PROPERTIES
    COMPILE_OPTIONS "-include;${CMAKE_CURRENT_SOURCE_DIR}/alloc_counter.h")
target_include_directories(test_items PRIVATE ../.. ../stubs ${LIBATOMVM_INCLUDE_PATH})
target_compile_definitions(test_items PRIVATE HAVE_ATOMIC)
target_compile_options(test_items PRIVATE -Wall -Wextra -Wno-unused-parameter)
if (ITEMS_SANITIZE)
    set(SANITIZE_FLAGS -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    target_compile_options(test_items PRIVATE ${SANITIZE_FLAGS})
    target_link_options(test_items PRIVATE ${SANITIZE_FLAGS})
    # READ_32_UNALIGNED left-shifts bytes >= 0x80 into the sign bit
    set_source_files_properties(../../dcs_lcd_draw.c PROPERTIES
        COMPILE_OPTIONS "-fno-sanitize=shift-base")
    # term_from_int left-shifts negative values
    set_source_files_properties(test_items.c PROPERTIES
        COMPILE_OPTIONS "-fno-sanitize=shift-base")
endif()
