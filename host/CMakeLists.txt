
cmake_minimum_required(VERSION 3.23)
set (CMAKE_CXX_STANDARD 11)

project(libhost VERSION 0.1)

set(SOURCES PacketSerial.h PacketSerial.cpp)

add_library( ${PROJECT_NAME} ${SOURCES})

#configure_file(version.h.in version.h)
#target_compile_definitions(${PROJECT_NAME} PUBLIC "TRJ_ENV_HOST")
#target_compile_definitions(${PROJECT_NAME} PUBLIC INTERRUPT_DELAY=4)
target_include_directories(${PROJECT_NAME} PUBLIC ../interface  ${PROJECT_SOURCE_DIR} )

