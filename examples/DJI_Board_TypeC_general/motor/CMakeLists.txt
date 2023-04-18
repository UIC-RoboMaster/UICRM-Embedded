project(example_motor_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_pwm
    TARGET DJI_Board_TypeC_general
    SOURCES pwm.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m2006
    TARGET DJI_Board_TypeC_general
    SOURCES m2006.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m3508
        TARGET DJI_Board_TypeC_general
        SOURCES m3508.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m2305
        TARGET DJI_Board_TypeC_general
        SOURCES m2305.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m6020
        TARGET DJI_Board_TypeC_general
        SOURCES m6020.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_servo_m2006
        TARGET DJI_Board_TypeC_general
        SOURCES servo_m2006.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_servo_m3508
        TARGET DJI_Board_TypeC_general
        SOURCES servo_m3508.cpp)