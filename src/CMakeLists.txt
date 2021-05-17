include(CodeCoverage)
append_coverage_compiler_flags()

add_library(portablesine throttle.cpp)
target_compile_definitions(portablesine PUBLIC "CONTROL=CTRL_FOC")
target_compile_definitions(portablesine PUBLIC "CTRL_SINE=0")
target_compile_definitions(portablesine PUBLIC "CTRL_FOC=1")

target_include_directories(portablesine PUBLIC "${PROJECT_SOURCE_DIR}/libopeninv/include")
target_include_directories(portablesine PRIVATE "${PROJECT_SOURCE_DIR}/include")

