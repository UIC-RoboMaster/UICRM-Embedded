project(Sentry_chassis ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        src/buzzer_task.cpp
        INCLUDES include)
