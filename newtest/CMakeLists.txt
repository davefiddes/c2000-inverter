enable_testing()

# Unit test the library
file(GLOB SRCS *.cpp)
add_executable(OpenInverterTest ${SRCS})

# Generic Google Test libraries
target_link_libraries(OpenInverterTest PUBLIC gtest_main)

include(GoogleTest)
gtest_discover_tests(OpenInverterTest)
