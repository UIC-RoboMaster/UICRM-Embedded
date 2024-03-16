project(DGStandard_chassis ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeA
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        # src/imu_task.cpp
        # src/gimbal_data.cpp
        src/chassis_task.cpp
        src/selftest_task.cpp
        src/buzzer_task.cpp
        INCLUDES include)
