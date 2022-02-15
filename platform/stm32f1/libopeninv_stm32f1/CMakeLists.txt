add_library(
  libopeninv_stm32f1
  src/anain.cpp
  src/digio.cpp
  src/param_save.cpp
  src/stm32_can.cpp
  src/stm32scheduler.cpp
  src/terminal.cpp
  src/terminalcommands.cpp)
target_link_libraries(libopeninv_stm32f1 global_options libopeninv)
target_include_directories(libopeninv_stm32f1
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# "Project" includes needed to customise the library build are pulled from the
# top level include directory
target_include_directories(libopeninv_stm32f1
                           PRIVATE "${CMAKE_SOURCE_DIR}/include")
