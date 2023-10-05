# crc
file(GLOB_RECURSE CRC_CHECK_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/crc_check/src/*.c)

file(GLOB_RECURSE DRIVER_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(DRIVER_SOURCES
        ${DRIVER_SOURCES_FILES}
        ${CRC_CHECK_SOURCES})
set(DRIVER_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/crc_check/include)

file(GLOB_RECURSE DJI_Board_TypeC_Specific_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/DJI_Board_TypeC/src/*.*)
set(DJI_Board_TypeC_Drivers_SOURCES
        ${DRIVER_SOURCES}
        ${DJI_Board_TypeC_Specific_SOURCES})
set(DJI_Board_TypeC_Drivers_INCLUDES
        ${DRIVER_INCLUDES}
        ${CMAKE_CURRENT_SOURCE_DIR}/DJI_Board_TypeC/include)
uicrm_add_board_specific_library(DJI_Board_TypeC_general_drivers
        TARGET DJI_Board_TypeC_general
        SOURCES
            ${DJI_Board_TypeC_Drivers_SOURCES}
        INCLUDES
            ${DJI_Board_TypeC_Drivers_INCLUDES}
        DEPENDS DJI_Board_TypeC_general_platform DJI_Board_TypeC_general_algorithm)
target_include_directories(DJI_Board_TypeC_general_drivers PUBLIC ${DJI_Board_TypeC_Drivers_INCLUDES})


file(GLOB_RECURSE DJI_Board_TypeA_Specific_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/DJI_Board_TypeA/src/*.*)
set(DJI_Board_TypeA_Drivers_SOURCES
        ${DRIVER_SOURCES}
        ${DJI_Board_TypeA_Specific_SOURCES})
set(DJI_Board_TypeA_Drivers_INCLUDES
        ${DRIVER_INCLUDES}
        ${CMAKE_CURRENT_SOURCE_DIR}/DJI_Board_TypeA/include)
uicrm_add_board_specific_library(DJI_Board_TypeA_general_drivers
        TARGET DJI_Board_TypeA_general
        SOURCES
        ${DJI_Board_TypeA_Drivers_SOURCES}
        INCLUDES
        ${DJI_Board_TypeA_Drivers_INCLUDES}
        DEPENDS DJI_Board_TypeA_general_platform DJI_Board_TypeA_general_algorithm)
target_include_directories(DJI_Board_TypeA_general_drivers PUBLIC ${DJI_Board_TypeA_Drivers_INCLUDES})


file(GLOB_RECURSE DM_MC01_Specific_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/DM_MC01/src/*.*)
set(DM_MC01_Drivers_SOURCES
        ${DRIVER_SOURCES}
        ${DM_MC01_Specific_SOURCES})
set(DM_MC01_Drivers_INCLUDES
        ${DRIVER_INCLUDES}
        ${CMAKE_CURRENT_SOURCE_DIR}/DM_MC01/include)
uicrm_add_board_specific_library(DM_MC01_general_drivers
        TARGET DM_MC01_general
        SOURCES
        ${DM_MC01_Drivers_SOURCES}
        INCLUDES
        ${DM_MC01_Drivers_INCLUDES}
        DEPENDS DM_MC01_general_platform DM_MC01_general_algorithm)
target_include_directories(DM_MC01_general_drivers PUBLIC ${DM_MC01_Drivers_INCLUDES})