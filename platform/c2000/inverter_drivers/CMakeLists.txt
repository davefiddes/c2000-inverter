add_library(
  inverter_drivers
  eeprom.cpp
  current.cpp
  encoder.cpp
  pmicspidriver.cpp
  pwmdriver.cpp
  gatedriver.cpp
  gatedriverinterface.cpp
  scheduler.cpp
  motoranalogcapture.cpp)
target_link_libraries(inverter_drivers global_options libopeninv device_support)
target_include_directories(inverter_drivers
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
