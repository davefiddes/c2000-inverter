add_library(inverter_drivers eeprom.cpp)
target_link_libraries(inverter_drivers global_options device_support)
target_include_directories(inverter_drivers
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
