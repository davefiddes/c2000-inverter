add_library(device_support source/device.c source/F2837xD_CodeStartBranch.asm)
target_compile_definitions(device_support PUBLIC CPU1 _LAUNCHXL_F28379D)
target_link_libraries(device_support global_options driverlib)
target_include_directories(device_support
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_options(device_support INTERFACE --entry_point=code_start)
