#
# This file is part of AtomVM.
#
# Copyright 2021-2023 Davide Bettio <davide@uninstall.it>
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
project (avm_sdl_display)

include_directories(${LIBATOMVM_INCLUDE_PATH})

find_package(SDL)
include_directories(${SDL_INCLUDE_DIR})
set(EXTRA_LIBS )

find_package(ZLIB)
if (ZLIB_FOUND)
    add_definitions(-DWITH_ZLIB)
else(ZLIB_FOUND)
    set(ZLIB_LIBRARIES "")
endif(ZLIB_FOUND)

if(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -ggdb")
endif()

set(CMAKE_SHARED_LIBRARY_PREFIX "")

add_library(avm_display_port_driver SHARED display.c ufontlib.c)
target_link_libraries(avm_display_port_driver ${SDL_LIBRARY} ${ZLIB_LIBRARIES})
set_property(TARGET avm_display_port_driver PROPERTY C_STANDARD 11)
set_property(TARGET avm_display_port_driver PROPERTY PREFIX "")
