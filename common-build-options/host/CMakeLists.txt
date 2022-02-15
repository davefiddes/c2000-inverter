# Host native build options

include(CheckSymbolExists)
include(CheckIncludeFiles)

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD
    20
    CACHE INTERNAL "")

check_include_files(numbers HAVE_NUMBERS_H)
if(HAVE_NUMBERS_H)
    check_symbol_exists(numbers pi HAVE_NUMBERS_PI)
    check_symbol_exists(numbers inv_sqrt3 HAVE_NUMBERS_INV_SQRT3)
endif()


# Set all of our common platform specific compiler options, defines and include
# paths
add_library(platform_options INTERFACE)

# Special library to add lastt when linking
add_library(link_last INTERFACE)

if("${CMAKE_${LANG}_COMPILER_ID}" MATCHES "(Apple)?[Cc]lang")
    target_compile_options(
        platform_options INTERFACE -ggdb3 -fprofile-arcs -ftest-coverage)
elseif(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(
        platform_options INTERFACE -ggdb3 -fprofile-arcs -ftest-coverage
                             -fprofile-abs-path)
    target_link_libraries(link_last INTERFACE -lgcov)
endif()


target_compile_options(platform_options INTERFACE -Wall -Wextra -Werror)

