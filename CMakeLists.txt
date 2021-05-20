include(CodeCoverage)
append_coverage_compiler_flags()
add_library(libopeninv src/errormessage.cpp src/params.cpp src/my_string.c src/printf.cpp src/my_fp.c src/sine_core.cpp src/picontroller.cpp src/foc.cpp src/fu.cpp)
target_compile_definitions(libopeninv PUBLIC "CONTROL=CTRL_FOC")
target_compile_definitions(libopeninv PUBLIC "CTRL_SINE=0")
target_compile_definitions(libopeninv PUBLIC "CTRL_FOC=1")

target_include_directories(libopeninv PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(libopeninv PRIVATE "${PROJECT_SOURCE_DIR}/include")
