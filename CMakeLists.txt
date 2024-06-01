cmake_minimum_required(VERSION 3.5)

set(USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(USE_UNITY_BUILDS "Enable unity build to improve build times" OFF)
option(TRACE_BUILD_TIME "Use -ftime-trace to generate build time trace" OFF)
option(ENABLE_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)

project(tyr)

# Raylib's vcpkg port forces sanitizers to be on in debug,
# we therefore need to force enable it in debug to avoid linking issues
add_compile_options($<$<OR:$<BOOL:${ENABLE_SANITIZERS}>,$<CONFIG:Debug>>:-fsanitize=address>)
add_link_options($<$<OR:$<BOOL:${ENABLE_SANITIZERS}>,$<CONFIG:Debug>>:-fsanitize=address>)

# undefined behavior sanitizer is not available on MSVC
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options("$<$<OR:$<BOOL:${ENABLE_SANITIZERS}>,$<CONFIG:Debug>>:-fno-omit-frame-pointer;-fsanitize=undefined>")
    add_link_options("$<$<OR:$<BOOL:${ENABLE_SANITIZERS}>,$<CONFIG:Debug>>:-fno-omit-frame-pointer;-fsanitize=undefined>")
endif ()

# Set warning levels
if (MSVC)
    # level 4 as the base
    add_compile_options(/W4)
    # but without some annoying warnings
    add_compile_options(/wd4244 /wd4267)
else ()
    # set the base
    add_compile_options(-Wall -Wextra -Wpedantic)
    # and remove annoying warnings
    add_compile_options(-Wno-error=sign-compare)
endif ()

# And treat them as error
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

if (${TRACE_BUILD_TIME})
    add_compile_options(-ftime-trace)
endif ()

# enable utf-8 support in msvc
if (MSVC)
    add_compile_options(/utf-8)
endif ()

# enable multithreaded build in VS
if (MSVC)
    add_compile_options(/MP)
endif ()

# Set data paths
add_compile_definitions(DATA_DIR="${CMAKE_SOURCE_DIR}/data")
add_compile_definitions(LOG_DIR="${CMAKE_SOURCE_DIR}/log")

add_subdirectory(source/vision)
add_subdirectory(source/referee)
add_subdirectory(source/sender)
add_subdirectory(source/soccer)
add_subdirectory(source/cli)
add_subdirectory(source/gui)

set(CPACK_GENERATOR ZIP)
set(CPACK_OUTPUT_FILE_PREFIX ${CMAKE_BINARY_DIR}/dist)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")
include(CPack)
