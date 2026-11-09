# 板卡无关的通信层：裁判系统线格式、分帧与赛季载荷解码。
# 只依赖 platform 提供的 bsp 接口，不接触具体外设引脚，也不依赖 third_party。
#
# 目录约定与 boards/drivers 一致：
#   referee/include/  对外头文件，该目录进 include path
#   referee/src/      实现文件

set(COMMUNICATION_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/crc.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/referee.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/seasons/2026/protocol.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/src/seasons/2027/protocol.cpp)

set(COMMUNICATION_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/referee/include)

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
