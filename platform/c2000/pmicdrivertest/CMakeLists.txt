# Test application to excercise the gate driver chips and PSU
add_executable(pmicdrivertest pmicdrivertest.cpp)
target_link_libraries(pmicdrivertest global_options inverter_drivers libopeninv device_support link_last)
