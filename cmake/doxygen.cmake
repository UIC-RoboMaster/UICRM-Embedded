# using doxygen to generate documents and using xdg-open to open index.html
find_program(DOXYGEN_EXE NAMES doxygen)
find_program(XDG_OPEN_EXE NAMES xdg-open)

# create document generating and viewing targets
if (DOXYGEN_EXE)
    # location of doxyfile and generated index.html
    set(DOXYFILE ${CMAKE_SOURCE_DIR}/Doxyfile)

    # generate documents
    add_custom_target(doc
        COMMAND ${DOXYGEN_EXE}
        DEPENDS ${DOXYFILE}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

endif()
