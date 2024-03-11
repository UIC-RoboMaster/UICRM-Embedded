
project(example_bmi088_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET ${BOARD_NAME}
        SOURCES main.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_AHRS
        TARGET ${BOARD_NAME}
        SOURCES AHRS.cpp)
