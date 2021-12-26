cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("Plugin for GazeboSim that receives the torque setpoints via UDP and applies them")

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#set(COMMUNICATIONS_LIBS zcm)

add_definitions(-std=c++17 -Wall -Wpedantic -I .. -O2 -I /usr/local/include -L /usr/local/lib)

# Build our plugin
add_library(biped_plugin SHARED biped_plugin.cc)
add_library(world_state_plugin SHARED world_state_plugin.cc)
add_library(foot_contact_plugin SHARED FootContactPlugin.cc)
target_link_libraries(biped_plugin world_state_plugin foot_contact_plugin ${GAZEBO_LIBRARIES})

#target_link_libraries(biped_plugin ${GAZEBO_LIBRARIES} ${COMMUNICATIONS_LIBS})