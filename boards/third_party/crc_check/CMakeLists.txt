
add_library(crc_check INTERFACE)
file(GLOB_RECURSE src/*.*)
target_include_directories(crc_check INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)