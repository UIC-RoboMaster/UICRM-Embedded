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

add_executable(automata_simple_fsm
        ${AUTOMATA_HOST_SOURCES}
        ${CMAKE_SOURCE_DIR}/examples/Host/Automata/simple_finite_state_machine.cpp
)

target_include_directories(automata_simple_fsm PRIVATE
        ${AUTOMATA_HOST_INCLUDES}
)

add_executable(automata_unit_test
        ${AUTOMATA_HOST_SOURCES}
        ${CMAKE_SOURCE_DIR}/examples/Host/Automata/unit_test.cpp
)

target_include_directories(automata_unit_test PRIVATE
        ${AUTOMATA_HOST_INCLUDES}
)