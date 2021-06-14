add_library(libopeninv src/errormessage.cpp src/params.cpp src/my_string.c src/printf.cpp src/my_fp.c src/sine_core.cpp src/picontroller.cpp src/foc.cpp src/fu.cpp)
target_link_libraries(libopeninv global_options)
target_include_directories(libopeninv PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(libopeninv PRIVATE "${PROJECT_SOURCE_DIR}/include")
