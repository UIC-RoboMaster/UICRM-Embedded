project(example_gimbal_${BOARD_NAME} ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET ${BOARD_NAME}
    SOURCES typeC.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_sbus
        TARGET ${BOARD_NAME}
        SOURCES typeC_sbus.cpp)