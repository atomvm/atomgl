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
