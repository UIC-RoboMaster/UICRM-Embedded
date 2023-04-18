project(example_protocol_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_host
        TARGET DJI_Board_TypeC_general
        SOURCES host.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_client
        TARGET DJI_Board_TypeC_general
        SOURCES client.cpp)