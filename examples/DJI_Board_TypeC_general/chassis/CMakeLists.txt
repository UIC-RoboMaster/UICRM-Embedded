project(example_chassis_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET ${BOARD_NAME}
    SOURCES main.cpp)


uicrm_add_arm_executable(${PROJECT_NAME}_sbus
        TARGET ${BOARD_NAME}
        SOURCES sbus.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_can_bridge
        TARGET ${BOARD_NAME}
        SOURCES can_bridge_chassis.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_power_data_collect
        TARGET ${BOARD_NAME}
        SOURCES power_data_collect.cpp
)

uicrm_add_arm_executable(${PROJECT_NAME}_power_limit_test
        TARGET ${BOARD_NAME}
        SOURCES power_limit_test.cpp
)