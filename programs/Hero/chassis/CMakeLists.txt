project(Hero_chassis ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DM_MC01_general
        SOURCES
        src/main.cpp
        src/buzzer_task.cpp
        INCLUDES include)
