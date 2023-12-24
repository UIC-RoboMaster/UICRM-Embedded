project(example_gimbal_${BOARD_NAME} ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET ${BOARD_NAME}
    SOURCES typeA.cpp)