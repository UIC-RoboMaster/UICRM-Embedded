
file(GLOB_RECURSE COMPONENTS_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(COMPONENTS_SOURCES ${COMPONENTS_SOURCES_FILES})
set(COMPONENTS_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include)


uicrm_add_board_specific_library(DJI_Board_TypeC_general_components
        TARGET DJI_Board_TypeC_general
        SOURCES ${COMPONENTS_SOURCES}
        INCLUDES ${COMPONENTS_INCLUDES}
        DEPENDS DJI_Board_TypeC_general_platform)
target_include_directories(DJI_Board_TypeC_general_components PUBLIC ${COMPONENTS_INCLUDES})
