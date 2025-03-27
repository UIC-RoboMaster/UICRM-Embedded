project(new_chassis_car3 ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        src/chassis_task.cpp
        src/buzzer_task.cpp
        INCLUDES include)
