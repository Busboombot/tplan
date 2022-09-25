cmake_minimum_required(VERSION 3.23)
set (CMAKE_CXX_STANDARD 11)

project (trajectory VERSION 0.0.1)

add_subdirectory(src)
add_subdirectory(src/test)
add_subdirectory(src/cli)
