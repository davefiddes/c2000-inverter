# STM32F1 build platforms (equivalent to current stm32_sine Makefile)

set(CMAKE_CXX_STANDARD
    14
    CACHE INTERNAL "")

# Set all of our common platform specific compiler options, defines and include
# paths
add_library(platform_options INTERFACE)
target_compile_definitions(platform_options INTERFACE "STM32F1")
target_compile_options(
  platform_options
  INTERFACE -Wall
            -Wextra
            -pedantic
            -fno-common
            -fno-builtin
            -mcpu=cortex-m3
            -mthumb
            $<$<COMPILE_LANGUAGE:C>:-std=gnu99>
            $<$<COMPILE_LANGUAGE:CXX>:-fno-builtin
            -fno-rtti
            -fno-exceptions
            -fno-unwind-tables>
            -ffunction-sections
            -fdata-sections
            -ggdb3)
target_link_options(
  platform_options INTERFACE -T ${CMAKE_SOURCE_DIR}/stm32_sine.ld -nostartfiles
  -Wl,--gc-sections,-Map,linker.map)

target_include_directories(platform_options
                           INTERFACE "${CMAKE_SOURCE_DIR}/libopencm3/include/")
target_link_libraries(platform_options INTERFACE -lopencm3_stm32f1)
target_link_directories(platform_options INTERFACE
                        "${CMAKE_SOURCE_DIR}/libopencm3/lib/")

# TODO: Figure out how to build libopencm3 automatically
