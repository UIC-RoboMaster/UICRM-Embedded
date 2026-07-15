# use clang-format to enforce coding styles
# NOTE: This project requires clang-format 18.
#   macOS:  brew install clang-format@18
#   Ubuntu: sudo apt install clang-format-18
find_program(CLANG_FORMAT_EXE NAMES clang-format)
# log the detected clang-format version and path
if (CLANG_FORMAT_EXE)
    execute_process(COMMAND ${CLANG_FORMAT_EXE} --version
            OUTPUT_VARIABLE CLANG_FORMAT_VERSION
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "Using clang-format: ${CLANG_FORMAT_EXE} (${CLANG_FORMAT_VERSION})")
    if (NOT CLANG_FORMAT_VERSION MATCHES "version 18")
        message(WARNING "clang-format version 18 is required to match CI. Detected: ${CLANG_FORMAT_VERSION}")
    endif()
endif()
# gather all source code
file(GLOB_RECURSE ALL_SOURCE_FILES
    ${CMAKE_SOURCE_DIR}/*.c
    ${CMAKE_SOURCE_DIR}/*.cc
    ${CMAKE_SOURCE_DIR}/*.cpp
    ${CMAKE_SOURCE_DIR}/*.h
    ${CMAKE_SOURCE_DIR}/*.hpp
)
# exclude build folder and board CubeMX generated code
list(FILTER ALL_SOURCE_FILES EXCLUDE REGEX .*/.*build.*/.*)
list(FILTER ALL_SOURCE_FILES EXCLUDE REGEX .*/boards/.*)

# create formatting helper targets
if (CLANG_FORMAT_EXE)
    # a third party clang-format python wrapper (respects .clang-format-ignore)
    set(RUN_CLANG_FORMAT ${CMAKE_SOURCE_DIR}/run-clang-format.py)
    if (CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
        message(STATUS "Current system is windows, clang-format should be proccess in dictionary.")
        # format code in place
        add_custom_target(format
            COMMAND python ${RUN_CLANG_FORMAT} --clang-format-executable ${CLANG_FORMAT_EXE} -i -r ${CMAKE_SOURCE_DIR}
            DEPENDS ${RUN_CLANG_FORMAT} 
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

        # check for format violations
        add_custom_target(check-format
            COMMAND python ${RUN_CLANG_FORMAT} --clang-format-executable ${CLANG_FORMAT_EXE} -r ${CMAKE_SOURCE_DIR}
            DEPENDS ${RUN_CLANG_FORMAT} 
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    else()
        # format code in place
        add_custom_target(format
            COMMAND python3 ${RUN_CLANG_FORMAT} --clang-format-executable ${CLANG_FORMAT_EXE} -i -r ${CMAKE_SOURCE_DIR}
            DEPENDS ${RUN_CLANG_FORMAT} 
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

        # check for format violations
        add_custom_target(check-format
            COMMAND python3 ${RUN_CLANG_FORMAT} --clang-format-executable ${CLANG_FORMAT_EXE} -r ${CMAKE_SOURCE_DIR}
            DEPENDS ${RUN_CLANG_FORMAT} 
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    endif(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
endif ()
