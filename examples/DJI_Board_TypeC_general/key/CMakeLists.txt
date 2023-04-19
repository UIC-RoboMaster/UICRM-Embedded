project(example_key_${BOARD_NAME} ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET ${BOARD_NAME}
        SOURCES main.cpp)