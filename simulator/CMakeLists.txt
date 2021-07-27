# Don't try to build any simulators for the STM32F1 platform as we don't have a
# working C-library currently
if(NOT PLATFORM STREQUAL "stm32f1")
  add_subdirectory(pwm)
endif()
