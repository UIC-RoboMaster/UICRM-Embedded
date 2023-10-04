
# printf
file(GLOB_RECURSE PRINTF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/printf/*.c)

file(GLOB_RECURSE STM32F4_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(STM32F4_SOURCES
        ${STM32F4_SOURCES_FILES}
        ${PRINTF_SOURCES})
set(STM32F4_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/printf/include)



uicrm_add_board_specific_library(DJI_Board_TypeC_general_platform
        TARGET DJI_Board_TypeC_general
        SOURCES ${STM32F4_SOURCES}
        INCLUDES ${STN32F4_INCLUDES}
)

target_include_directories(DJI_Board_TypeC_general_platform PUBLIC ${STM32F4_INCLUDES})