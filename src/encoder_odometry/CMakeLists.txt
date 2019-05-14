cmake_minimum_required(VERSION 2.8.3)
project(encoder_odometry)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  encoder_msgs
)

## This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    encoder_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

add_compile_options(-std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable

add_library(odometry_wheels src/odometry_wheels.cpp)
add_executable(odometry_wheels_main src/odometry_wheels_main.cpp)

## Add cmake target dependencies of the executable
add_dependencies(odometry_wheels_main ${catkin_EXPORTED_TARGETS})

## Specify libraries to link libraries and executable targets against
target_link_libraries(odometry_wheels_main odometry_wheels ${catkin_LIBRARIES})

catkin_add_gtest(test_odometry_wheels src/test_odometry_wheels.cpp)
target_link_libraries(test_odometry_wheels odometry_wheels ${catkin_LIBRARIES})

add_executable(simulated_wheels src/simulated_wheels.cpp)
target_link_libraries(simulated_wheels ${catkin_LIBRARIES})
