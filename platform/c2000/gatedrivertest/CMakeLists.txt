# Test application to excercise the gate driver chips and PSU
add_executable(gatedrivertest gatedrivertest.cpp)
target_link_libraries(gatedrivertest global_options device_support link_last)
