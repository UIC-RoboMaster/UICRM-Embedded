project(example_referee_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET ${BOARD_NAME}
        SOURCES main.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_rc
        TARGET ${BOARD_NAME}
        SOURCES rc.cpp)