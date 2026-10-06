project(example_chassis_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_mecanum
    TARGET ${BOARD_NAME}
    SOURCES mecanum_example.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_omni
        TARGET ${BOARD_NAME}
        SOURCES omni_example.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_sbus
        TARGET ${BOARD_NAME}
        SOURCES sbus.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_can_bridge
        TARGET ${BOARD_NAME}
        SOURCES can_bridge_chassis.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_can_bridge_sender
        TARGET ${BOARD_NAME}
        SOURCES can_bridge_chassis_sender.cpp)
