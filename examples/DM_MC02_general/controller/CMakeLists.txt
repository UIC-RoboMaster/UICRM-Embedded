project(example_controller_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_dt7
    TARGET ${BOARD_NAME}
    SOURCES dt7.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_vt13
    TARGET ${BOARD_NAME}
    SOURCES vt13.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_fs_i6x
    TARGET ${BOARD_NAME}
    SOURCES fs_i6x.cpp)
