# Host native build options

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 17 CACHE INTERNAL "")

# Set all of our common platform specific compiler options, defines and include paths
add_library(platform_options INTERFACE)
target_compile_options(platform_options INTERFACE "-ggdb3" "-fprofile-arcs" "-ftest-coverage" "-fprofile-abs-path")
target_compile_options(platform_options INTERFACE "-Wall" "-Wextra")
