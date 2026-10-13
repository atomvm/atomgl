#
# This file is part of AtomGL.
#
# Copyright 2026 Davide Bettio <davide@uninstall.it>
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

# Host unit tests for the platform independent AtomGL modules.
#
#   cmake -S tests/host -B build-tests && cmake --build build-tests && ctest --test-dir build-tests

cmake_minimum_required(VERSION 3.13)
project(atomgl_host_tests C)

option(ATOMGL_TESTS_SANITIZE "Build the tests with AddressSanitizer and UBSan" ON)

set(ATOMGL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# display_items.h includes libAtomVM headers; default to the AtomVM checkout
# this component normally lives in (src/platforms/esp32/components/atomgl).
set(LIBATOMVM_INCLUDE_PATH "${ATOMGL_DIR}/../../../../libAtomVM" CACHE PATH "Path to libAtomVM headers")
if (NOT EXISTS "${LIBATOMVM_INCLUDE_PATH}/term.h")
    message(FATAL_ERROR "libAtomVM headers not found, set LIBATOMVM_INCLUDE_PATH")
endif()

add_library(atomgl_test_options INTERFACE)
target_compile_options(atomgl_test_options INTERFACE -Wall -Wextra)
target_include_directories(atomgl_test_options INTERFACE ${ATOMGL_DIR})
if (ATOMGL_TESTS_SANITIZE)
    target_compile_options(atomgl_test_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(atomgl_test_options INTERFACE -fsanitize=address,undefined)
endif()

enable_testing()

add_executable(test_display_damage test_display_damage.c ${ATOMGL_DIR}/display_damage.c)
target_link_libraries(test_display_damage atomgl_test_options)
set_property(TARGET test_display_damage PROPERTY C_STANDARD 11)
add_test(NAME display_damage COMMAND test_display_damage)

add_executable(test_display_diff test_display_diff.c ${ATOMGL_DIR}/display_diff.c
    ${ATOMGL_DIR}/display_damage.c ${ATOMGL_DIR}/shape.c ${ATOMGL_DIR}/font_data.c)
target_include_directories(test_display_diff PRIVATE ${LIBATOMVM_INCLUDE_PATH})
target_compile_definitions(test_display_diff PRIVATE AVM_TASK_DRIVER_ENABLED HAVE_ATOMIC)
target_link_libraries(test_display_diff atomgl_test_options)
set_property(TARGET test_display_diff PROPERTY C_STANDARD 11)
add_test(NAME display_diff COMMAND test_display_diff)
