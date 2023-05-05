project(example_protocol_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_host
        TARGET ${BOARD_NAME}
        SOURCES host.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_client
        TARGET ${BOARD_NAME}
        SOURCES client.cpp)