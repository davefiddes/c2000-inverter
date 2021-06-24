# C2000 build platform

set(CMAKE_CXX_STANDARD
    98
    CACHE INTERNAL "")

set(CMAKE_C_STANDARD
    11
    CACHE INTERNAL "")

# Set all of our common platform specific compiler options, defines and include
# paths
add_library(platform_options INTERFACE)
target_compile_definitions(platform_options INTERFACE C2000 CPU1
                                                      _LAUNCHXL_F28379D)
target_compile_options(
  platform_options
  INTERFACE -v28
            --abi=eabi
            -ml
            -mt
            --cla_support=cla1
            --float_support=fpu32
            --emit_warnings_as_errors
            --tmu_support=tmu0
            --vcu_support=vcu2
            --advice:performance=all)
target_link_options(
  platform_options
  INTERFACE
  -v28
  --abi=eabi
  -ml
  -mt
  --cla_support=cla1
  --float_support=fpu32
  --tmu_support=tmu0
  --vcu_support=vcu2
  --advice:performance=all
  -mlinker.map
  --stack_size=0x100
  --warn_sections
  --reread_libs
  --diag_wrap=off
  --display_error_number
  --entry_point=code_start
  --rom_model
  "${CMAKE_CURRENT_SOURCE_DIR}/2837xD_RAM_lnk_cpu1.cmd"
  -llibc.a)
