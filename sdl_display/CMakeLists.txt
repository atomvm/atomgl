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

option(AVM_DISABLE_SMP "Disable SMP." OFF)
option(AVM_DISABLE_TASK_DRIVER "Disable task driver support." OFF)

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

add_library(avm_display_port_driver SHARED display.c ufontlib.c ../image_helpers.c ../spng.c)

if (AVM_DISABLE_SMP)
    target_compile_definitions(avm_display_port_driver PUBLIC AVM_NO_SMP)
endif()
if (NOT AVM_DISABLE_TASK_DRIVER)
    target_compile_definitions(avm_display_port_driver PUBLIC AVM_TASK_DRIVER_ENABLED)
endif()

include(CheckIncludeFile)
CHECK_INCLUDE_FILE(stdatomic.h STDATOMIC_INCLUDE)
include(CheckCSourceCompiles)
check_c_source_compiles("
    #include <stdatomic.h>
    int main() {
        _Static_assert(ATOMIC_POINTER_LOCK_FREE == 2, \"Expected ATOMIC_POINTER_LOCK_FREE to be equal to 2\");
    }
" ATOMIC_POINTER_LOCK_FREE_IS_TWO)
if (ATOMIC_POINTER_LOCK_FREE_IS_TWO)
    target_compile_definitions(avm_display_port_driver PUBLIC HAVE_ATOMIC)
endif()

target_link_libraries(avm_display_port_driver ${SDL_LIBRARY} ${ZLIB_LIBRARIES})
set_property(TARGET avm_display_port_driver PROPERTY C_STANDARD 11)
set_property(TARGET avm_display_port_driver PROPERTY PREFIX "")
