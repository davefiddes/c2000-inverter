# Test application to dump the contents of the inverter EEPROM for posterity
add_executable(dumpeeprom dumpeeprom.cpp)
target_link_libraries(dumpeeprom global_options inverter_drivers device_support link_last)
