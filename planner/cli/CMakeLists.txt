
cmake_minimum_required(VERSION 3.23)

project(cli LANGUAGES CXX)
set (CMAKE_CXX_STANDARD 11)

find_package(Boost   COMPONENTS program_options filesystem system  REQUIRED)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Could not find boost!")
endif()

add_executable(planner planner.cpp )
target_link_libraries(planner PUBLIC ${Boost_LIBRARIES} libplanner libhost libinterface )
target_include_directories(planner PUBLIC ${libinterface_SOURCE_DIR} ${Boost_INCLUDE_DIRS} )

add_executable(stepper stepper.cpp )
target_link_libraries(stepper PRIVATE  libplanner libhost libinterface libinterface)
target_include_directories(stepper PUBLIC ${libinterface_SOURCE_DIR} ${Boost_INCLUDE_DIRS} )
