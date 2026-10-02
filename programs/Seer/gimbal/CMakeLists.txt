project(Seer_gimbal ASM C CXX)

option(SEER_ENABLE_MINIPC "Enable MiniPC navigation and auto aim for Seer gimbal" OFF)
set(SEER_MINIPC_SOURCES)
set(SEER_MINIPC_DEFINES)
if(SEER_ENABLE_MINIPC)
    list(APPEND SEER_MINIPC_SOURCES src/minipc_task.cpp)
    list(APPEND SEER_MINIPC_DEFINES HAS_MINIPC)
endif()

uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        src/public_port.cpp
        src/imu_task.cpp
        src/gimbal_task.cpp
        src/shoot_task.cpp
        src/remote_task.cpp
        src/chassis_task.cpp
        src/referee_task.cpp
#        src/ui_task.cpp
        src/buzzer_task.cpp
        ${SEER_MINIPC_SOURCES}
        INCLUDES include
        DEFINES ${SEER_MINIPC_DEFINES}
#        DEFINES HAS_REFEREE
)
