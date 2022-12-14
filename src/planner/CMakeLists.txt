
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
target_compile_definitions(${PROJECT_NAME} PUBLIC "TRJ_DEBUG")
target_compile_definitions(${PROJECT_NAME} PUBLIC INTERRUPT_DELAY=4)

message("FastCRC source dir: ${libfastcrc_SOURCE_DIR}")

target_link_libraries(${PROJECT_NAME} PRIVATE libfastcrc)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${libfastcrc_SOURCE_DIR}/src
        )