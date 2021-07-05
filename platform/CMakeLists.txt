# All platform specific binaries go in here but until they do it is optional to
# have a platform directory
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PLATFORM})
  add_subdirectory(${PLATFORM})
endif()
