
# printf
file(GLOB_RECURSE PRINTF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/printf/src/*.c)

file(GLOB_RECURSE STM32F1_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(STM32F1_SOURCES
        ${STM32F1_SOURCES_FILES}
        ${PRINTF_SOURCES})
set(STM32F1_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/printf/include)

uicrm_add_board_specific_library(F103_Nano_general_platform
        TARGET F103_Nano_general
        SOURCES ${STM32F1_SOURCES}
        INCLUDES ${STN32F1_INCLUDES}
)
target_include_directories(F103_Nano_general_platform PUBLIC ${STM32F1_INCLUDES})