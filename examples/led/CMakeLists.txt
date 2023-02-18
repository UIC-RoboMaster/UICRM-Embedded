project(example_led ASM C CXX)

#uicrm_add_arm_executable(${PROJECT_NAME}_typeA
#    TARGET DJI_Board_TypeA
#    SOURCES typeA.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_typeC
    TARGET DJI_Board_TypeC_general
    SOURCES typeC.cpp)
