include(CTest)
find_package(Catch2 3 REQUIRED)
include(Catch)

add_subdirectory(planner)
add_subdirectory(host)
add_subdirectory(test/unit)
add_subdirectory(test/system)


set (CMAKE_CXX_STANDARD 11)


