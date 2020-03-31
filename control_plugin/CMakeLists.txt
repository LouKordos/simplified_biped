cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#set(COMMUNICATIONS_LIBS zcm)

# Build our plugin
add_library(biped_plugin SHARED biped_plugin.cc)
target_link_libraries(biped_plugin ${GAZEBO_LIBRARIES})

#target_link_libraries(biped_plugin ${GAZEBO_LIBRARIES} ${COMMUNICATIONS_LIBS})