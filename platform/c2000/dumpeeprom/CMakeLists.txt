# Test application to dump the contents of the inverter EEPROM for posterity
add_executable(dumpeeprom dumpeeprom.cpp eeprom.cpp)
target_link_libraries(dumpeeprom global_options device_support link_last)
