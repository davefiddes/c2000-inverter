# Pull in our platform specific options
add_subdirectory(${PLATFORM})

# Set all of our common compiler options, defines and include paths
add_library(global_options INTERFACE)
target_compile_definitions(global_options INTERFACE "CONTROL=CTRL_FOC")
target_compile_definitions(global_options INTERFACE "CTRL_SINE=0")
target_compile_definitions(global_options INTERFACE "CTRL_FOC=1")
target_link_libraries(global_options INTERFACE platform_options)
