add_library(portablesine throttle.cpp)
target_link_libraries(portablesine global_options libopeninv)
target_include_directories(libopeninv PUBLIC "${PROJECT_SOURCE_DIR}/include")

