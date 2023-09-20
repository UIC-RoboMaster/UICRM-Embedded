
file(GLOB_RECURSE DRIVER_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(DRIVER_SOURCES ${DRIVER_SOURCES_FILES})
set(DRIVER_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include)


uicrm_add_board_specific_library(DJI_Board_TypeC_general_drivers
        TARGET DJI_Board_TypeC_general
        SOURCES ${DRIVER_SOURCES}
        INCLUDES ${DRIVER_INCLUDES}
        DEPENDS DJI_Board_TypeC_general_platform DJI_Board_TypeC_general_algorithm crc_check)
target_include_directories(DJI_Board_TypeC_general_drivers PUBLIC ${DRIVER_INCLUDES})
