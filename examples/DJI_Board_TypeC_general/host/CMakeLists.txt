project(example_host_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_uart
        TARGET ${BOARD_NAME}
        SOURCES host_uart.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_usb
        TARGET ${BOARD_NAME}
        SOURCES host_usb.cpp)
