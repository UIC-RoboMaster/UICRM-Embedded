
project(example_automata_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_fsm_led
        TARGET ${BOARD_NAME}
        SOURCES fsm_led.cpp)