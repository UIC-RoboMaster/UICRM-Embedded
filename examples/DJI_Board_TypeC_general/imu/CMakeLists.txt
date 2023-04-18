project(example_imu_${BOARD_NAME} ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET DJI_Board_TypeC_general
    SOURCES typeC.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_oled
        TARGET DJI_Board_TypeC_general
        SOURCES typeC_oled.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_usb
        TARGET DJI_Board_TypeC_general
        SOURCES typeC_usb.cpp)