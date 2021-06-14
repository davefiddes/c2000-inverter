include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/965f8ecbfd8b91bbd4f5ee4914c028660bb89029.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Unit test the library
file(GLOB SRCS *.cpp)
add_executable(OpenInverterTest ${SRCS})

target_link_libraries(OpenInverterTest PUBLIC global_options libopeninv portablesine)

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
