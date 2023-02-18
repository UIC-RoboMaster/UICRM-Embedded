cmake_minimum_required(VERSION 3.8)

include(cmake/arm_toolchain.cmake)
include(cmake/build_helper.cmake)
include(cmake/clang_format.cmake)
include(cmake/doxygen.cmake)

project(uicrm)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_subdirectory(boards)
add_subdirectory(shared)
#add_subdirectory(programs)
add_subdirectory(examples)