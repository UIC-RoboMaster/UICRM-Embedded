project(example_motor_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_pwm
    TARGET ${BOARD_NAME}
    SOURCES pwm.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m2006
    TARGET ${BOARD_NAME}
    SOURCES m2006.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m3508
        TARGET ${BOARD_NAME}
        SOURCES m3508.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m2305
        TARGET ${BOARD_NAME}
        SOURCES m2305.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m6020
        TARGET ${BOARD_NAME}
        SOURCES m6020.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_servo_m2006
        TARGET ${BOARD_NAME}
        SOURCES servo_m2006.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_servo_m3508
        TARGET ${BOARD_NAME}
        SOURCES servo_m3508.cpp)