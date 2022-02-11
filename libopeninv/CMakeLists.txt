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
  src/sine_core.cpp
  src/crc8.cpp)
target_link_libraries(libopeninv global_options)
target_include_directories(libopeninv
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# "Project" includes needed to customise the library build are pulled from the
# top level include directory
target_include_directories(libopeninv PRIVATE "${CMAKE_SOURCE_DIR}/include")
