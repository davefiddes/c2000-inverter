# Build a simple LED blinky for the LAUNCHXL-F28379D to prove the toolchain
# works
add_executable(blinky blinky.cpp)
target_link_libraries(blinky global_options device_support -llibc.a)
