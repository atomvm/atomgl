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
project (atomgl_damage_tests C)

set(CMAKE_C_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

if (NOT LIBATOMVM_INCLUDE_PATH)
    message(FATAL_ERROR "Set LIBATOMVM_INCLUDE_PATH to AtomVM's src/libAtomVM (headers only, nothing is linked)")
endif()

option(DAMAGE_SANITIZE "Build with ASan and UBSan, aborting on the first report" ON)

add_executable(test_damage test_damage.c ../../sdl_display/damage.c)
target_include_directories(test_damage PRIVATE ../../sdl_display ${LIBATOMVM_INCLUDE_PATH})
target_compile_definitions(test_damage PRIVATE HAVE_ATOMIC)
target_compile_options(test_damage PRIVATE -Wall -Wextra -Wno-unused-parameter)
if (DAMAGE_SANITIZE)
    set(SANITIZE_FLAGS -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    target_compile_options(test_damage PRIVATE ${SANITIZE_FLAGS})
    target_link_options(test_damage PRIVATE ${SANITIZE_FLAGS})
endif()
