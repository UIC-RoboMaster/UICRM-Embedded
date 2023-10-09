# use clang-format to enforce coding styles
find_program(CLANG_FORMAT_EXE NAMES 
    clang-format-10
    clang-format-9
    clang-format-8
    clang-format)

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
    # download a third party clang format python wrapper
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
            COMMAND python3 ${RUN_CLANG_FORMAT} --clang-format-executable ${CLANG_FORMAT_EXE} -i ${ALL_SOURCE_FILES}
            DEPENDS ${RUN_CLANG_FORMAT} 
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

        # check for format violations
        add_custom_target(check-format
            COMMAND python3 ${RUN_CLANG_FORMAT} --clang-format-executable ${CLANG_FORMAT_EXE} ${ALL_SOURCE_FILES}
            DEPENDS ${RUN_CLANG_FORMAT} 
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    endif(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
endif ()
