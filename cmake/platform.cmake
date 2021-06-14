if (NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED PLATFORM)
    # If we have a platform and haven't manually defined a toolchain file then
    # automatically pull in the toolchain definition we are to use
    set(CMAKE_TOOLCHAIN_FILE    ${PROJECT_SOURCE_DIR}/common-build-options/${PLATFORM}/toolchain.cmake)
endif ()

if (NOT DEFINED PLATFORM)
    message("No PLATFORM defined. Configuring for native host platform")
    set(PLATFORM "host" CACHE STRING "Target platform")
endif ()

