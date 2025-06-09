cmake_minimum_required(VERSION 3.0.2)
project(robotiq_gazebo)

## Compiler flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  gazebo_ros
  control_toolbox
)

## Find Gazebo
find_package(gazebo REQUIRED)

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)

## Catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    gazebo_mimic_joint_plugin
    gazebo_disable_link_plugin
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    control_toolbox
  DEPENDS 
    Boost
    gazebo
)

## Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Compiler flags
add_definitions(${GAZEBO_CXX_FLAGS})

## Mimic Joint Plugin
add_library(gazebo_mimic_joint_plugin src/mimic_joint_plugin.cpp)
target_link_libraries(gazebo_mimic_joint_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Disable Link Plugin
add_library(gazebo_disable_link_plugin src/disable_link_plugin.cpp)
target_link_libraries(gazebo_disable_link_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Install targets
install(TARGETS 
    gazebo_mimic_joint_plugin
    gazebo_disable_link_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install plugin XML or launch files if needed
# install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
# install(DIRECTORY worlds/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
