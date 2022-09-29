
cmake_minimum_required(VERSION 3.23)
set (CMAKE_CXX_STANDARD 11)

project(libplanner VERSION 0.1)

add_library( ${PROJECT_NAME}
        Hardware.cpp
        Hardware.h
        block.cpp
        block.h
        col.h
        config.h
        const.h
        debug.cpp
        debug.h
        ipacketserial.h
        joint.cpp
        joint.h
        loop.cpp
        loop.h
        messageprocessor.cpp
        messageprocessor.h
        move.cpp
        move.h
        planner.cpp
        planner.h
        planner_types.h
        segment.cpp
        segment.h
        stepper.cpp
        stepper.h
        types.cpp
        types.h
        util.cpp
        util.h
        version.h
        )

configure_file(version.h.in version.h)
target_compile_definitions(${PROJECT_NAME} PUBLIC "TRJ_ENV_HOST")
target_compile_definitions(${PROJECT_NAME} PUBLIC INTERRUPT_DELAY=4)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}
        )