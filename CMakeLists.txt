cmake_minimum_required(VERSION 3.14)
project(my_project)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/965f8ecbfd8b91bbd4f5ee4914c028660bb89029.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_subdirectory(libopeninv)
add_subdirectory(src)

# The unit tests
add_subdirectory(newtest)