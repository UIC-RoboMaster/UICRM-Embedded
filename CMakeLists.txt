cmake_minimum_required(VERSION 3.8)

option(UICRM_BUILD_ARM "Build firmware for ARM target" ON)

if(UICRM_BUILD_ARM)
    include(cmake/arm_toolchain.cmake)
    include(cmake/build_helper.cmake)
    include(cmake/clang_format.cmake)
    include(cmake/doxygen.cmake)
else ()
    include(cmake/host_toolchain.cmake)
    include(cmake/host_debug.cmake)
endif ()

project(uicrm)

if (UICRM_BUILD_ARM)
    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

    add_subdirectory(boards)
    add_subdirectory(programs)
    add_subdirectory(examples)
else ()
    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
endif ()


