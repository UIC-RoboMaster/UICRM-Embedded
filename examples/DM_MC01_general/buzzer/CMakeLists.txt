
project(example_buzzer_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET DM_MC01_general
    SOURCES main.cpp)
