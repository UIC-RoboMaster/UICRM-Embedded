project(new_gimbal_car3 ASM C CXX)

file(GLOB_RECURSE TINYML_SOURCES_FILES ${CMAKE_CURRENT_SOURCE_DIR}/TinyML/src/*.cpp)
set(TINYML_SOURCES
        ${TINYML_SOURCES_FILES})
set(TINYML_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/TinyML/inc)

uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        src/public_port.cpp
        src/gimbal_task.cpp
        src/minipc_task.cpp
#        src/shoot_task_CAN.cpp
        src/shoot_task.cpp
        src/remote_task.cpp
        src/chassis_task.cpp
        src/referee_task.cpp
        src/ui_task.cpp
        src/buzzer_task.cpp
        src/imu_task.cpp
        src/debug_model.cpp
        ${TINYML_SOURCES}

        INCLUDES
        include
        ${TINYML_INCLUDES})