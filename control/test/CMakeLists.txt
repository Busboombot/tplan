
cmake_minimum_required(VERSION 3.23)

project(mp_test LANGUAGES CXX)
set (CMAKE_CXX_STANDARD 11)

include(CTest)
find_package(Catch2 3 REQUIRED)
include(Catch)
enable_testing()


find_package(Boost COMPONENTS system filesystem REQUIRED)
if(NOT Boost_FOUND)
    message(FATAL_ERROR "Could not find boost!")
endif()


set( SOURCES_TESTS
        test.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES_TESTS} )

target_link_libraries(${PROJECT_NAME} PRIVATE Catch2::Catch2WithMain libcontrol libhost libinterface  ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${Boost_INCLUDE_DIRS}
        )

catch_discover_tests(${PROJECT_NAME})
