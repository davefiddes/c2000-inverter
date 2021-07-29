# Main inverter application
add_executable(inverter inverter_main.cpp c2000current.cpp c2000encoder.cpp
                        c2000pwmdriver.cpp)
target_link_libraries(inverter global_options portablesine libopeninv
                      device_support link_last)
