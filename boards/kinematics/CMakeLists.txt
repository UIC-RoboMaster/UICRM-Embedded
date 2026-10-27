set(KINEMATICS_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(uicrm_kinematics INTERFACE)
target_include_directories(uicrm_kinematics INTERFACE ${KINEMATICS_INCLUDES})
