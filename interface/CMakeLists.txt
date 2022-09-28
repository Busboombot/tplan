
cmake_minimum_required(VERSION 3.23)
set (CMAKE_CXX_STANDARD 11)

project(libinterface VERSION 0.1)

set(SOURCES
        col.h
        json.hpp
        Hardware.h
        Hardware.cpp
        iplanner.h
        trj_config.h
        trj_const.h
        trj_move.cpp
        trj_move.h
        trj_types.h
        trj_types.cpp
        trj_debug.h
        trj_debug.cpp
        ipacketserial.h
        trj_util.cpp
        trj_util.h)

add_library( ${PROJECT_NAME} ${SOURCES})

#configure_file(version.h.in version.h)
target_compile_definitions(${PROJECT_NAME} PUBLIC "TRJ_ENV_HOST")
target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR} )