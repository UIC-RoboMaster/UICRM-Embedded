file(GLOB_RECURSE AHRS_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/MahonyAHRS/src/*.c)
set(AHRS_SOURCES ${AHRS_SOURCES_FILES})
set(AHRS_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/MahonyAHRS/include)

file(GLOB_RECURSE ALGORITHM_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.*)
set(ALGORITHM_SOURCES
        ${ALGORITHM_SOURCES_FILES}
        ${AHRS_SOURCES_FILES})
set(ALGORITHM_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${AHRS_INCLUDES})



uicrm_add_board_specific_library(DJI_Board_TypeC_general_algorithm
        TARGET DJI_Board_TypeC_general
        SOURCES ${ALGORITHM_SOURCES}
        INCLUDES ${ALGORITHM_INCLUDES}
        DEPENDS DJI_Board_TypeC_general_platform)
target_include_directories(DJI_Board_TypeC_general_algorithm PUBLIC ${ALGORITHM_INCLUDES})

uicrm_add_board_specific_library(DJI_Board_TypeA_general_algorithm
        TARGET DJI_Board_TypeA_general
        SOURCES ${ALGORITHM_SOURCES}
        INCLUDES ${ALGORITHM_INCLUDES}
        DEPENDS DJI_Board_TypeA_general_platform)
target_include_directories(DJI_Board_TypeA_general_algorithm PUBLIC ${ALGORITHM_INCLUDES})

uicrm_add_board_specific_library(DM_MC01_general_algorithm
        TARGET DM_MC01_general
        SOURCES ${ALGORITHM_SOURCES}
        INCLUDES ${ALGORITHM_INCLUDES}
        DEPENDS DM_MC01_general_platform)
target_include_directories(DM_MC01_general_algorithm PUBLIC ${ALGORITHM_INCLUDES})

uicrm_add_board_specific_library(F103_Nano_general_algorithm
        TARGET F103_Nano_general
        SOURCES ${ALGORITHM_SOURCES}
        INCLUDES ${ALGORITHM_INCLUDES}
        DEPENDS F103_Nano_general_platform)
target_include_directories(F103_Nano_general_algorithm PUBLIC ${ALGORITHM_INCLUDES})