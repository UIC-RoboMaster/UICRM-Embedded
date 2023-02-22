project(example_imu ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeC
    TARGET DJI_Board_TypeC_general
    SOURCES typeC.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_typeC_oled
        TARGET DJI_Board_TypeC_general
        SOURCES typeC_oled.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_typeC_usb
        TARGET DJI_Board_TypeC_general
        SOURCES typeC_usb.cpp)