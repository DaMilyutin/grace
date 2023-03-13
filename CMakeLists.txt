cmake_minimum_required(VERSION 3.13)

project(grace)

enable_testing()

option(GRACE_ENABLE_TESTS "No tests for ylems" OFF)

set (CMAKE_CXX_STANDARD          20)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/_lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/_bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/_bin)

add_definitions(-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS)

set(GCC_CLANG_COMPILER_OPTIONS_CXX -std=c++20 -fPIC -Wall -fno-rtti)
set(GCC_CLANG_COMPILER_OPTIONS_CXX ${GCC_CLANG_COMPILER_OPTIONS_CXX}
	-Wall -Wextra -Wpedantic
	-Wno-missing-field-initializers
	-Wno-implicit-fallthrough
)
if(NOT ANDROID_ABI OR ANDROID_ABI MATCHES "x86.*")
	set(GCC_CLANG_COMPILER_OPTIONS_CXX ${GCC_CLANG_COMPILER_OPTIONS_CXX} -msse2)
endif()

add_compile_options(
	"$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:${GCC_CLANG_COMPILER_OPTIONS_CXX}>"
	"$<$<CXX_COMPILER_ID:MSVC>:-W4;-wd4512;-wd4355>"
	"$<$<CXX_COMPILER_ID:MSVC>:$<$<CONFIG:Release,RelWithDebInfo>:-Ox;-Ob2>>"
	"$<$<CXX_COMPILER_ID:MSVC>:$<$<EQUAL:4,${CMAKE_SIZEOF_VOID_P}>:-arch:SSE2>>"
)

if (GNU)
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
endif()

if (NOT TARGET ylems)
    option(YLEMS_NO_TESTS "No tests for ylems" ON)
    add_subdirectory(libraries/ylems)
endif()

add_library(grace INTERFACE)
target_include_directories(grace INTERFACE ./include/.)

if(GRACE_ENABLE_TESTS)
	add_subdirectory(tests/unit)
	add_subdirectory(tests/benchmark)
endif()






