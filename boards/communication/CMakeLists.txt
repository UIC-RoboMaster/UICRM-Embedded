set(COMMUNICATION_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/crc.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/frame.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/referee.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/seasons/2026/protocol.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/seasons/2027/protocol.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/host/src/host.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/host/src/link.cpp)

set(COMMUNICATION_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/include
        ${CMAKE_CURRENT_SOURCE_DIR}/host/include)

# 与 boards/drivers 相同的板卡清单，保证任一板卡程序都能取到本层。
set(COMMUNICATION_BOARDS
        DJI_Board_TypeC_general
        DJI_Board_TypeA_general
        DM_MC01_general
        DM_MC02_general
        F103_Nano_general
        BulletExchanger_F103)

foreach(BOARD IN LISTS COMMUNICATION_BOARDS)
    uicrm_add_board_specific_library(${BOARD}_communication
            TARGET ${BOARD}
            SOURCES ${COMMUNICATION_SOURCES}
            INCLUDES ${COMMUNICATION_INCLUDES}
            DEPENDS ${BOARD}_platform)
    target_include_directories(${BOARD}_communication PUBLIC ${COMMUNICATION_INCLUDES})
endforeach()
