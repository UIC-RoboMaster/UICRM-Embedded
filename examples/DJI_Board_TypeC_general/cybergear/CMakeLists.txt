project(example_cybergear_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_mit
    TARGET ${BOARD_NAME}
    SOURCES mit.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_current
        TARGET ${BOARD_NAME}
        SOURCES current.cpp)