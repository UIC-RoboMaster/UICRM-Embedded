project(Neo ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        gimbal/src/main.cpp
        gimbal/src/public_port.cpp
        gimbal/src/imu_task.cpp
        gimbal/src/gimbal_data.cpp
        gimbal/src/gimbal_task.cpp
        gimbal/src/shoot_task.cpp
        gimbal/src/remote_task.cpp
        gimbal/src/chassis_task.cpp
        gimbal/src/referee_task.cpp
        gimbal/src/ui_task.cpp
        gimbal/src/buzzer_task.cpp
        INCLUDES gimbal/include)

uicrm_add_arm_executable(${PROJECT_NAME}_chassis_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        chassis/can_bridge_chassis.cpp
)
