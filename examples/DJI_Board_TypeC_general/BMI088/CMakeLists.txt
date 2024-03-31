
project(example_bmi088_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET ${BOARD_NAME}
        SOURCES main.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_with_ist8310
        TARGET ${BOARD_NAME}
        SOURCES with_ist8310.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_AHRS
        TARGET ${BOARD_NAME}
        SOURCES AHRS.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_QEKF
        TARGET ${BOARD_NAME}
        SOURCES QEKF.cpp)
