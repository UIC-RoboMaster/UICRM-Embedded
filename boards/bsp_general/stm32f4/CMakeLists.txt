

file(GLOB_RECURSE STM32F4_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(STM32F4_SOURCES ${STM32F4_SOURCES_FILES})
set(STM32F4_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include)


uicrm_add_board_specific_library(DJI_Board_TypeC_general_platform
        TARGET DJI_Board_TypeC_general
        SOURCES ${STM32F4_SOURCES}
        INCLUDES ${STN32F4_INCLUDES}
        DEPENDS printf)

target_include_directories(DJI_Board_TypeC_general_platform PUBLIC ${STM32F4_INCLUDES})