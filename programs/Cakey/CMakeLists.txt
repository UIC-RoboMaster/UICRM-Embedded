project(Cakey ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeA
        TARGET DJI_Board_TypeA_general
        SOURCES
        src/main.cpp
        src/public_port.cpp
        src/imu_task.cpp
        src/gimbal_task.cpp
        src/minipc_task.cpp
        src/shoot_task.cpp
        src/remote_task.cpp
        src/chassis_task.cpp
        src/referee_task.cpp
        src/ui_task.cpp
        src/buzzer_task.cpp
        INCLUDES include)
