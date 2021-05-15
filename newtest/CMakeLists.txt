enable_testing()

# Unit test the library
file(GLOB SRCS *.cpp)
add_executable(OpenInverterTest ${SRCS})
add_compile_definitions("CONTROL=CTRL_FOC")
add_compile_definitions("CTRL_SINE=0")
add_compile_definitions("CTRL_FOC=1")

target_include_directories(OpenInverterTest PUBLIC "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(OpenInverterTest PUBLIC libopeninv)

# Create a code coverage utility target - assumes that gcov and lcov are available
# TODO: Test for these utilities
include(CodeCoverage)
append_coverage_compiler_flags()
setup_target_for_coverage_lcov(
    NAME coverage_OpenInverterTest
    EXECUTABLE OpenInverterTest
    OUTPUT coverage_OpenInverterTest_dir
    EXCLUDE "/usr/include/*" "build/_deps/*"
    )

# Generic Google Test libraries
target_link_libraries(OpenInverterTest PUBLIC gtest_main gmock_main)

include(GoogleTest)
gtest_discover_tests(OpenInverterTest)
