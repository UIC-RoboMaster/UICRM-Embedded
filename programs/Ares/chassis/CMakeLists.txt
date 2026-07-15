project(Ares_chassis ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        src/buzzer_task.cpp
        INCLUDES include)

uicrm_add_arm_executable(${PROJECT_NAME}_typeC_example
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/chassis_example.cpp
        src/buzzer_task.cpp
        INCLUDES include)
