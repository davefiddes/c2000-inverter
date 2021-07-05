cmake_minimum_required(VERSION 3.19)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Determine what platform we are targetting with this build and set up
# toolchains if required
include(platform)

project(OpenInverter LANGUAGES C CXX ASM)

add_subdirectory(common-build-options)
add_subdirectory(libopeninv)
add_subdirectory(src)
add_subdirectory(platform)

# The unit tests only if we are building natively
if (PLATFORM STREQUAL "host")
    add_subdirectory(test)
endif ()