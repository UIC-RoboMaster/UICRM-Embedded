set(AUTOMATA_HOST_SOURCES
        ${CMAKE_SOURCE_DIR}/boards/algorithm/src/StateAutomatas.cpp
        ${CMAKE_SOURCE_DIR}/boards/drivers/src/Automata/AutomataInputManagement.cpp
        ${CMAKE_SOURCE_DIR}/boards/drivers/src/Automata/AutomataInputBase.cpp
        ${CMAKE_SOURCE_DIR}/boards/drivers/src/Automata/AutomataInputRemote.cpp
)

set(AUTOMATA_HOST_INCLUDES
        ${CMAKE_SOURCE_DIR}/boards/drivers/include
        ${CMAKE_SOURCE_DIR}/boards/algorithm/include
)

add_executable(simple_fsm
        ${AUTOMATA_HOST_SOURCES}
        ${CMAKE_SOURCE_DIR}/examples/Host/Automata/simple_finite_state_machine.cpp
)

target_include_directories(simple_fsm PRIVATE
        ${AUTOMATA_HOST_INCLUDES}
)

add_executable(no_input_fsm
        ${AUTOMATA_HOST_SOURCES}
        ${CMAKE_SOURCE_DIR}/examples/Host/Automata/no_input.cpp
)

target_include_directories(no_input_fsm PRIVATE
        ${AUTOMATA_HOST_INCLUDES}
)