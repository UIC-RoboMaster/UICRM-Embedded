
project(example_imu_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET ${BOARD_NAME}
        SOURCES main.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_calibrate
        TARGET ${BOARD_NAME}
        SOURCES calibrate.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_poseEstimate
        TARGET ${BOARD_NAME}
        SOURCES poseEstimate.cpp)