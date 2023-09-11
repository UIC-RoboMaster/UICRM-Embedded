
add_library(printf INTERFACE)
file(GLOB_RECURSE src/*.*)
target_include_directories(printf INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)