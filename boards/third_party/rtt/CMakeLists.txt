add_library(rtt INTERFACE)

set(RTT_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/rtt)

# The assembly file needs preprocessing. GCC on windows does not recognize .S extension, so we copy it to .sx
file(COPY ${RTT_SRC_DIR}/RTT/SEGGER_RTT_ASM_ARMv7M.S
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/SEGGER_RTT_ASM_ARMv7M.sx
)

target_sources(rtt INTERFACE
    ${RTT_SRC_DIR}/RTT/SEGGER_RTT.c
    ${RTT_SRC_DIR}/RTT/SEGGER_RTT_printf.c
    ${CMAKE_CURRENT_BINARY_DIR}/SEGGER_RTT_ASM_ARMv7M.sx
)

target_include_directories(rtt INTERFACE
    ${RTT_SRC_DIR}/RTT
    ${RTT_SRC_DIR}/Config
)