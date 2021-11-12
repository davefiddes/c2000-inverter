# Main inverter application
add_executable(inverter inverter_main.cpp)
target_link_libraries(inverter global_options portablesine libopeninv
                      inverter_drivers device_support link_last)
