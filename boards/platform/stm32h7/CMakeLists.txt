
# printf
file(GLOB_RECURSE PRINTF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/printf/src/*.c)

file(GLOB_RECURSE STM32H7_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(STM32H7_SOURCES
        ${STM32H7_SOURCES_FILES}
        ${PRINTF_SOURCES})
set(STM32H7_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/printf/include)

uicrm_add_board_specific_library(DM_MC02_general_platform
        TARGET DM_MC02_general
        SOURCES ${STM32H7_SOURCES}
        INCLUDES ${STN32H7_INCLUDES}
)
target_include_directories(DM_MC02_general_platform PUBLIC ${STM32H7_INCLUDES})