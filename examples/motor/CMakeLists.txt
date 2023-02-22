project(example_motor ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_pwm
    TARGET DJI_Board_TypeC_general
    SOURCES pwm.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m2006
    TARGET DJI_Board_TypeC_general
    SOURCES m2006.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m6020
        TARGET DJI_Board_TypeC_general
        SOURCES m6020.cpp)