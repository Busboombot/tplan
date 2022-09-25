cmake_minimum_required(VERSION 3.23)
set (CMAKE_CXX_STANDARD 11)
project (trajectory VERSION 0.1.0)

include(CTest)
find_package(Catch2 3 REQUIRED)
include(Catch)

add_subdirectory(planner)
add_subdirectory(planner/test)
add_subdirectory(planner/cli)

add_subdirectory(messageprocessor)
#add_subdirectory(cmdloop)

