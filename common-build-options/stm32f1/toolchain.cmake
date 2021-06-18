set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Without that flag CMake is not able to pass test compilation check
set(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

# toolchain paths
find_program(ARM_AR         NAMES arm-none-eabi-ar${CMAKE_EXECUTABLE_SUFFIX})
find_program(ARM_AS         NAMES arm-none-eabi-gcc${CMAKE_EXECUTABLE_SUFFIX})
find_program(ARM_CC         NAMES arm-none-eabi-gcc${CMAKE_EXECUTABLE_SUFFIX})
find_program(ARM_CXX        NAMES arm-none-eabi-g++${CMAKE_EXECUTABLE_SUFFIX})
find_program(ARM_LD         NAMES arm-none-eabi-ld${CMAKE_EXECUTABLE_SUFFIX})
find_program(ARM_OBJCOPY    NAMES arm-none-eabi-objcopy${CMAKE_EXECUTABLE_SUFFIX})
find_program(ARM_RANLIB     NAMES arm-none-eabi-ranlib${CMAKE_EXECUTABLE_SUFFIX})
find_program(ARM_SIZE       NAMES arm-none-eabi-size${CMAKE_EXECUTABLE_SUFFIX})
find_program(ARM_STRIP      NAMES arm-none-eabi-strip${CMAKE_EXECUTABLE_SUFFIX})

# set executables settings
set(CMAKE_C_COMPILER    ${ARM_CC})
set(CMAKE_CXX_COMPILER  ${ARM_CXX})
set(CMAKE_ASM_COMPILER  ${ARM_AS})
set(CMAKE_AR            ${ARM_AR})
set(CMAKE_OBJCOPY       ${ARM_OBJCOPY} CACHE INTERNAL "")
set(CMAKE_OBJDUMP       ${ARM_OBJDUMP} CACHE INTERNAL "")
set(CMAKE_SIZE          ${ARM_SIZE} CACHE INTERNAL "")
set(CMAKE_STRIP         ${ARM_STRIP} CACHE INTERNAL "")
set(CMAKE_LINKER        ${ARM_LD} CACHE INTERNAL "")

set(CMAKE_C_FLAGS_DEBUG             "-Og -ggdb3" CACHE INTERNAL "")
set(CMAKE_C_FLAGS_RELEASE           "-Os -ggdb3 -DNDEBUG" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_C_FLAGS_DEBUG}" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_C_FLAGS_RELEASE}" CACHE INTERNAL "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
