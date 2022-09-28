
cmake_minimum_required(VERSION 3.23)
set (CMAKE_CXX_STANDARD 11)

project(libplanner VERSION 0.1)

add_library( ${PROJECT_NAME}
        trj_segment.cpp
        trj_block.cpp
        trj_joint.cpp
        planner.cpp
        stepper.cpp stepper.h)

configure_file(version.h.in version.h)
target_compile_definitions(${PROJECT_NAME} PUBLIC "TRJ_ENV_HOST")
target_compile_definitions(${PROJECT_NAME} PUBLIC INTERRUPT_DELAY=4)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${libinterface_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
        )