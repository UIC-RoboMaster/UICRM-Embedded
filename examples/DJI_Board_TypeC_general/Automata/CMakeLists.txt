
project(example_automata_${BOARD_NAME} ASM C CXX)

#uicrm_add_arm_executable(${PROJECT_NAME}_fsm_led
#        TARGET ${BOARD_NAME}
#        SOURCES fsm_led.cpp)
#
uicrm_add_arm_executable(${PROJECT_NAME}_performance
        TARGET ${BOARD_NAME}
        SOURCES performance_test.cpp)

#uicrm_add_arm_executable(${PROJECT_NAME}_switch_performance
#        TARGET ${BOARD_NAME}
#        SOURCES switch_performance.cpp)