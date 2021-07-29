add_executable(pwmsimulator pwmsimulator.cpp simulatorcurrent.cpp
                            simulatorencoder.cpp simulatorpwmdriver.cpp)
target_link_libraries(pwmsimulator global_options portablesine libopeninv link_last)
