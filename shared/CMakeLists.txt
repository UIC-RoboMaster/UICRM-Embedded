# Eigen interface library
add_library(eigen3 INTERFACE)
target_include_directories(eigen3 SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.3.9)

file(GLOB_RECURSE BSP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/bsp/*.cpp)
file(GLOB_RECURSE LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/libraries/*.cpp)
file(GLOB_RECURSE THIRD_PARTY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/*.c)

set(UICRM_SOURCES ${BSP_SOURCES} ${LIB_SOURCES} ${THIRD_PARTY_SOURCES})
set(UICRM_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party)


uicrm_add_board_specific_library(DJI_Board_TypeC_general_shared
    TARGET DJI_Board_TypeC_general
    SOURCES ${UICRM_SOURCES}
    INCLUDES ${UICRM_INCLUDES}
    DEPENDS eigen3)

uicrm_add_board_specific_library(DM_MC01_general_shared
    TARGET DM_MC01_general
    SOURCES ${UICRM_SOURCES}
    INCLUDES ${UICRM_INCLUDES}
    DEPENDS eigen3)

uicrm_add_board_specific_library(DJI_Board_TypeA_general_shared
        TARGET DJI_Board_TypeA_general
        SOURCES ${UICRM_SOURCES}
        INCLUDES ${UICRM_INCLUDES}
        DEPENDS eigen3)
