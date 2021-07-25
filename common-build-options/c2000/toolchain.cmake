###############################################################################
# cl2000 - Toolchain File
# Derived from https://gist.github.com/farrrb/032dcb907a0787ede67173fe68f97915
# from F. Zahn - 2018
###############################################################################

# set target system
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION  1)
set(CMAKE_SYSTEM_PROCESSOR          c2000)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# toolchain paths
find_program(TI_CC              NAMES   cl2000${CMAKE_EXECUTABLE_SUFFIX})
find_program(TI_CXX             NAMES   cl2000${CMAKE_EXECUTABLE_SUFFIX})
find_program(TI_AS              NAMES   cl2000${CMAKE_EXECUTABLE_SUFFIX})
find_program(TI_AR              NAMES   ar2000${CMAKE_EXECUTABLE_SUFFIX})
find_program(TI_OBJCOPY         NAMES   ofd2000${CMAKE_EXECUTABLE_SUFFIX})
find_program(TI_OBJDUMP         NAMES   hex2000${CMAKE_EXECUTABLE_SUFFIX})
find_program(TI_SIZE            NAMES   size2000${CMAKE_EXECUTABLE_SUFFIX})
find_program(TI_LD              NAMES   cl2000${CMAKE_EXECUTABLE_SUFFIX})

# set executables settings
set(CMAKE_C_COMPILER    ${TI_CC})
set(CMAKE_CXX_COMPILER  ${TI_CXX})
set(CMAKE_ASM_COMPILER  ${TI_AS})
set(CMAKE_AR            ${TI_AR})
set(CMAKE_OBJCOPY       ${TI_OBJCOPY})
set(CMAKE_OBJDUMP       ${TI_OBJDUMP})
set(CMAKE_SIZE          ${TI_SIZE})
set(CMAKE_LINKER        ${TI_LD})

#Figure out where the standard libraries and includes are automatically (like most other compilers do automatically)
cmake_path(GET TI_CC PARENT_PATH TI_COMPILER_BIN_PATH)
cmake_path(GET TI_COMPILER_BIN_PATH PARENT_PATH TI_COMPILER_PATH)

message( "TI Compile Path: ${TI_COMPILER_PATH}" )

set(CMAKE_C_FLAGS                   "-v28 --abi=eabi -I${TI_COMPILER_PATH}/include" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS                 "${CMAKE_C_FLAGS}" CACHE INTERNAL "")
set(CMAKE_ASM_FLAGS                 "${CMAKE_C_FLAGS}" CACHE INTERNAL "")

set(CMAKE_C_FLAGS_DEBUG             "-O0" CACHE INTERNAL "")
set(CMAKE_C_FLAGS_RELEASE           "-O4" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_C_FLAGS_DEBUG}" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_C_FLAGS_RELEASE}" CACHE INTERNAL "")

set(CMAKE_EXE_LINKER_FLAGS          "-I${TI_COMPILER_PATH}/include -I${TI_COMPILER_PATH}/lib")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)