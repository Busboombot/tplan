
cmake_minimum_required(VERSION 3.23)
set (CMAKE_CXX_STANDARD 11)

project(libcontrol VERSION 0.1)

set(SOURCES messageprocessor.cpp loop.cpp ../target/TargetHardware.cpp ../target/TargetHardware.h)

add_library( ${PROJECT_NAME} ${SOURCES})

#configure_file(version.h.in version.h)
target_compile_definitions(${PROJECT_NAME} PUBLIC "TRJ_ENV_HOST")
target_compile_definitions(${PROJECT_NAME} PUBLIC INTERRUPT_DELAY=4)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${libinterface_SOURCE_DIR}
        ${libhost_SOURCE_DIR}
        ${libplanner_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR} )

add_subdirectory(test)