add_library(portablesine throttle.cpp)
target_link_libraries(portablesine global_options libopeninv)
target_include_directories(libopeninv PUBLIC "${CMAKE_SOURCE_DIR}/include")

if(PLATFORM STREQUAL "stm32f1")
  add_executable(
    stm32_sine
    hwinit.cpp
    inc_encoder.cpp
    stm32_sine.cpp
    stm32pwmdriver.cpp
    temp_meas.cpp
    terminal_prj.cpp
    vehiclecontrol.cpp)
  target_link_libraries(stm32_sine global_options)
  target_link_libraries(stm32_sine libopeninv libopeninv_stm32f1 portablesine)
endif()
