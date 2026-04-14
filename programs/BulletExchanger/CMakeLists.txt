project(BulletExchanger ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_F103
        TARGET F103_Nano_general
        SOURCES
            src/main.cpp
)
