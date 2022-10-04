
cmake_minimum_required(VERSION 3.23)

project(devel_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)

include(CTest)
find_package(Catch2 3 REQUIRED)
include(Catch)
enable_testing()

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
    message(FATAL_ERROR "Could not find boost!")
endif ()

set(SOURCES_TESTS
        test.cpp
        test_blink.cpp
        test_misc.cpp
        ../test_lib.cpp
        )

add_executable(test_devel ${SOURCES_TESTS})

target_link_libraries(test_devel PRIVATE Catch2::Catch2WithMain libplanner libhost ${Boost_LIBRARIES})

target_include_directories(test_devel PUBLIC
        ${libhost_SOURCE_DIR}
        ${libfastcrc_SOURCE_DIR}/src
        ${Boost_INCLUDE_DIRS}
        ../
        )

catch_discover_tests(test_devel)
