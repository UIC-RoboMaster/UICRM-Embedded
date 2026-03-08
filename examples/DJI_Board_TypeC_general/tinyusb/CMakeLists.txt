cmake_minimum_required(VERSION 3.20)

#include(${CMAKE_CURRENT_SOURCE_DIR}/../../../hw/bsp/family_support.cmake)

#project(cdc_msc C CXX ASM)
project(example_tinyusb_${BOARD_NAME}_typec ASM C CXX)

# Checks this example is valid for the family and initializes the project
#family_initialize_project(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR})

# Espressif has its own cmake build system
#if(FAMILY STREQUAL "espressif")
#  return()
#endif()
#
#if (RTOS STREQUAL zephyr)
#  set(EXE_NAME app)
#else()
#  set(EXE_NAME ${PROJECT_NAME})
#  add_executable(${EXE_NAME})
#endif()

#set(EXE_NAME ${PROJECT_NAME})
#add_executable(${EXE_NAME})

set(tinyusb_src_dir ${PROJECT_SOURCE_DIR}/../../../boards/third_party/tinyusb/src)

set(SOURCE_FILES
    ${tinyusb_src_dir}/portable/synopsys/dwc2/dcd_dwc2.c
    ${tinyusb_src_dir}/portable/synopsys/dwc2/hcd_dwc2.c
    ${tinyusb_src_dir}/portable/synopsys/dwc2/dwc2_common.c
#    ${tinyusb_src_dir}/../hw/bsp/stm32f4/family.c
#D:\Handle\dev\RM\UICRM-Embedded\boards\third_party\tinyusb\hw\bsp\stm32n6\family.c
#    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/msc_disk.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_descriptors.cpp
  )

set(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${tinyusb_src_dir}
    ${tinyusb_src_dir}/../hw
  )

target_compile_definitions(${BOARD_NAME} PUBLIC USE_TINYUSB)
target_include_directories(${BOARD_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src )

tinyusb_target_add(${BOARD_NAME})

uicrm_add_arm_executable(${PROJECT_NAME}
    TARGET ${BOARD_NAME}
    SOURCES ${SOURCE_FILES}
    INCLUDES ${INCLUDE_DIRS}
)


## Example source
#target_sources(${EXE_NAME} PRIVATE
#
#  )

## Example include
#target_include_directories(${EXE_NAME} PUBLIC
#  ${CMAKE_CURRENT_SOURCE_DIR}/src
#  )

# Configure compilation flags and libraries for the example without RTOS.
# See the corresponding function in hw/bsp/FAMILY/family.cmake for details.
#family_configure_device_example(${EXE_NAME} ${RTOS})


#target_sources(${BOARD_NAME} PUBLIC
##        ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/family.c
##        ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../board.c
#        ${PROJECT_SOURCE_DIR}/boards/third_party/tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c
#        ${PROJECT_SOURCE_DIR}/boards/third_party/tinyusb/src/portable/synopsys/dwc2/hcd_dwc2.c
#        ${PROJECT_SOURCE_DIR}/boards/third_party/tinyusb/src/portable/synopsys/dwc2/dwc2_common.c
##        ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
#)
#target_include_directories(${BOARD_NAME} PUBLIC
#        ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
#        ${PROJECT_SOURCE_DIR}/boards/third_party/tinyusb/
##        ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
#)


#uicrm_add_arm_executable(${PROJECT_NAME}
#        TARGET ${BOARD_NAME}
#        SOURCES main.cpp
#        INCLUDES
#)
#
