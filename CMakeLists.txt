cmake_minimum_required(VERSION 3.23)

project (trajectory VERSION 0.1.0)

include(CTest)
find_package(Catch2 3 REQUIRED)
include(Catch)

message(STATUS "DOXYGEN_EXECUTABLE = '${DOXYGEN_EXECUTABLE}'")
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/config-file.in.doxygen)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/config-file.doxygen)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

add_subdirectory(src)

