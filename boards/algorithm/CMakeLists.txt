file(GLOB_RECURSE ALGORITHM_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(ALGORITHM_SOURCES ${ALGORITHM_SOURCES_FILES})
set(ALGORITHM_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include)


uicrm_add_board_specific_library(DJI_Board_TypeC_general_algorithm
        TARGET DJI_Board_TypeC_general
        SOURCES ${ALGORITHM_SOURCES}
        INCLUDES ${ALGORITHM_INCLUDES}
        DEPENDS DJI_Board_TypeC_general_platform crc_check)
target_include_directories(DJI_Board_TypeC_general_algorithm PUBLIC ${ALGORITHM_INCLUDES})