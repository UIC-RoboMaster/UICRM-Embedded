
file(GLOB_RECURSE COMPONENTS_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(COMPONENTS_SOURCES ${COMPONENTS_SOURCES_FILES})
set(COMPONENTS_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include)


uicrm_add_board_specific_library(DJI_Board_TypeC_general_components
        TARGET DJI_Board_TypeC_general
        SOURCES ${COMPONENTS_SOURCES}
        INCLUDES ${COMPONENTS_INCLUDES}
        DEPENDS DJI_Board_TypeC_general_drivers DJI_Board_TypeC_general_algorithm DJI_Board_TypeC_general_platform)
target_include_directories(DJI_Board_TypeC_general_components PUBLIC ${COMPONENTS_INCLUDES})

uicrm_add_board_specific_library(DJI_Board_TypeA_general_components
        TARGET DJI_Board_TypeA_general
        SOURCES ${COMPONENTS_SOURCES}
        INCLUDES ${COMPONENTS_INCLUDES}
        DEPENDS DJI_Board_TypeA_general_drivers DJI_Board_TypeA_general_algorithm DJI_Board_TypeA_general_platform)
target_include_directories(DJI_Board_TypeA_general_components PUBLIC ${COMPONENTS_INCLUDES})

uicrm_add_board_specific_library(DM_MC01_general_components
        TARGET DM_MC01_general
        SOURCES ${COMPONENTS_SOURCES}
        INCLUDES ${COMPONENTS_INCLUDES}
        DEPENDS DM_MC01_general_drivers DM_MC01_general_algorithm DM_MC01_general_platform)
target_include_directories(DM_MC01_general_components PUBLIC ${COMPONENTS_INCLUDES})

uicrm_add_board_specific_library(F103_Nano_general_components
        TARGET F103_Nano_general
        SOURCES ${COMPONENTS_SOURCES}
        INCLUDES ${COMPONENTS_INCLUDES}
        DEPENDS F103_Nano_general_drivers F103_Nano_general_algorithm F103_Nano_general_platform)
target_include_directories(F103_Nano_general_components PUBLIC ${COMPONENTS_INCLUDES})