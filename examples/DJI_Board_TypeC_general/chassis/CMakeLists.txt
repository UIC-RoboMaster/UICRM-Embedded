project(example_chassis_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET ${BOARD_NAME}
    SOURCES main.cpp)


uicrm_add_arm_executable(${PROJECT_NAME}_sbus
        TARGET ${BOARD_NAME}
        SOURCES sbus.cpp)