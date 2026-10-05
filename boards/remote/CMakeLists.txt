set(UICRM_REMOTE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/protocols/dbus_receiver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/protocols/sbus_receiver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/protocols/refereerc_receiver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/controller/fs_i6x_receiver.cpp)

# The board-specific driver libraries compile these sources with their own target settings.
set(UICRM_REMOTE_SOURCES ${UICRM_REMOTE_SOURCES} PARENT_SCOPE)
set(UICRM_REMOTE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
