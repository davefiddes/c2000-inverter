# Test application to excercise the gate driver chips and PSU
add_executable(gatedrivertest gatedrivertest.cpp gatedriver.cpp gatedriverspi.cpp)
target_link_libraries(gatedrivertest global_options inverter_drivers libopeninv device_support link_last)
