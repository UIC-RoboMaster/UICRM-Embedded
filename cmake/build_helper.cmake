## uicrm_add_arm_executable(<name>
#                         TARGET <board_specific_driver_library>
#                         SOURCES <src1>.c [<src2>.cc <src3>.s ...]
#                         [DEPENDS <dep1> ...]
#                         [INCLUDES <inc1> ...])
#
#   helper function for generating arm executables <name>.elf, <name>.bin, <name>.hex
#   `flash-<name>` (and optionally `debug-<name>` in debug build) will be created as
#   shortcuts command to flash via stlink-utils and launching gdb
#
#   e.g. uicrm_add_arm_executable(hero TARGET DJI_Board_TypeA ...) creates shortcuts
#   target named `flash-hero` and `debug-hero`. Running the targets in command line
#   is as easy as
#
#   `make flash-hero` and / or `make debug-hero`
#
function(uicrm_add_arm_executable name)
    cmake_parse_arguments(ARG "" "TARGET" "SOURCES;INCLUDES;DEPENDS;DEFINES" ${ARGN})
    set(HEX_FILE ${CMAKE_CURRENT_BINARY_DIR}/${name}.hex)
    set(BIN_FILE ${CMAKE_CURRENT_BINARY_DIR}/${name}.bin)
    set(MAP_FILE ${CMAKE_CURRENT_BINARY_DIR}/${name}.map)

    add_executable(${name}.elf ${ARG_SOURCES})
    target_link_libraries(${name}.elf
        PRIVATE ${ARG_DEPENDS} ${ARG_TARGET} ${ARG_TARGET}_platform ${ARG_TARGET}_algorithm ${ARG_TARGET}_drivers ${ARG_TARGET}_components)
    
    target_include_directories(${name}.elf PRIVATE ${ARG_INCLUDES})
    target_compile_definitions(${name}.elf PRIVATE ${ARG_DEFINES})
    target_link_options(${name}.elf PRIVATE -Wl,--print-memory-usage,-Map=${MAP_FILE})

    find_program(ARM_SIZE arm-none-eabi-size REQUIRED)
    find_program(ARM_OBJCOPY arm-none-eabi-objcopy REQUIRED)
    add_custom_command(TARGET ${name}.elf POST_BUILD
        COMMAND ${ARM_SIZE} ${name}.elf
        COMMAND ${ARM_OBJCOPY} -Oihex $<TARGET_FILE:${name}.elf> ${HEX_FILE}
        COMMAND ${ARM_OBJCOPY} -Obinary $<TARGET_FILE:${name}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}")

#    add_custom_target(flash-${name}
#        COMMAND st-flash --reset write ${BIN_FILE} 0x8000000
#        DEPENDS ${name}.elf)

#    if (NOT CMAKE_BUILD_TYPE STREQUAL "Release")
#        find_program(ARM_GDB arm-none-eabi-gdb REQUIRED)
#        add_custom_target(debug-${name}
#            COMMAND ${ARM_GDB} $<TARGET_FILE:${name}.elf>
#            DEPENDS ${name}.elf)
#    endif()
endfunction(uicrm_add_arm_executable)

## uicrm_add_board_specific_library(<name>
#                                 TARGET <board_specific_driver_library>
#                                 SOURCES <src1>.c [<src2>.cc <src3>.s ...]
#                                 [INCLUDES <inc1> ...]
#                                 [DEPENDS <dep1> ...])
#
#   helper function for generating a board specific static library
#   see shared/CMakeLists.txt for example usage
function(uicrm_add_board_specific_library name)
    cmake_parse_arguments(ARG "" "TARGET" "SOURCES;INCLUDES;DEPENDS" ${ARGN})
    add_library(${name} OBJECT ${ARG_SOURCES})
    target_link_libraries(${name}
        PUBLIC ${ARG_TARGET}_interface
        PRIVATE ${ARG_DEPENDS})
    target_include_directories(${name} PUBLIC ${ARG_INCLUDES})
endfunction(uicrm_add_board_specific_library)
