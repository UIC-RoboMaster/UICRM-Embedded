project(example_laser ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET DJI_Board_TypeC_general
    SOURCES main.cpp)
