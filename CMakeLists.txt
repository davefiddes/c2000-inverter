add_library(
  libopeninv
  src/errormessage.cpp
  src/foc.cpp
  src/fu.cpp
  src/my_fp.c
  src/my_string.c
  src/params.cpp
  src/picontroller.cpp
  src/printf.cpp
  src/sine_core.cpp)
target_link_libraries(libopeninv global_options)
target_include_directories(libopeninv
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# "Project" includes needed to customise the library build are pulled from the
# top level include directory
target_include_directories(libopeninv PRIVATE "${CMAKE_SOURCE_DIR}/include")

if(PLATFORM STREQUAL "stm32f1")
  add_library(
    libopeninv_stm32f1
    src/anain.cpp
    src/digio.cpp
    src/param_save.cpp
    src/stm32_can.cpp
    src/stm32scheduler.cpp
    src/terminal.cpp
    src/terminalcommands.cpp)
  target_link_libraries(libopeninv_stm32f1 global_options)
  target_include_directories(libopeninv_stm32f1
                             PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
  # "Project" includes needed to customise the library build are pulled from the
  # top level include directory
  target_include_directories(libopeninv_stm32f1
                             PRIVATE "${CMAKE_SOURCE_DIR}/include")
endif()
