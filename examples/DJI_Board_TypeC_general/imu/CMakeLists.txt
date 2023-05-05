project(example_imu_${BOARD_NAME} ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET ${BOARD_NAME}
    SOURCES typeC.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_oled
        TARGET ${BOARD_NAME}
        SOURCES typeC_oled.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_usb
        TARGET ${BOARD_NAME}
        SOURCES typeC_usb.cpp)