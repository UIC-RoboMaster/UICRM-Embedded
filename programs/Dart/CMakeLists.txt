project(Dart ASM C CXX)


uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        INCLUDES include)
