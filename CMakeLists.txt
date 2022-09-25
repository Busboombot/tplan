cmake_minimum_required(VERSION 3.23)
set (CMAKE_CXX_STANDARD 11)
project (trajectory VERSION 0.1.0)

include(CTest)
find_package(Catch2 3 REQUIRED)
include(Catch)

add_subdirectory(src)
add_subdirectory(src/test)
add_subdirectory(src/cli)


