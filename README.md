# Description

This implements a simple wheel encoder-based odometric system.  Wheel encoder values are sent on two separate topics,
and the odometric node does not assume synchronization, and will interpolate as needed.  There is an assumption, however, that odometric data will be interleaved.  The odometry node publishes data at a specified rate to the topic /odom.

# Files

The main code is divided as follows:
* encoder_odometry/include/odometry_wheel_helpers.h - helper functions.
* encoder_odometry/include/odometry_wheels.h - Thread-safe class for the node, which depends on ROS messages, but not any direct communication with the middleware.
* encoder_odometry/src/odometry_wheels.cpp - Class implementation.
* encoder_odometry/src/odometry_wheels_main.cpp - Lightweight ROS wrapper class and main.
* encoder_odometry/src/test_odometry_wheels.cpp - Unit test for helpers and class.
* encoder_odometry/src/simulated_wheels.cpp - Test program that makes wheels go straight, turn left, and turn right, forever.

# Building/running

To build:

* Source /opt/ros/kinetic/setup.bash (replace kinetic with your installed distro)
* Run catkin_make from top directory.

To run:
* In one terminal, run: roscore.
* In another, run: rosrun encoder_odometry simualted_wheels
* In another, run: rosrun encoder_odometry odometry_wheels
* Then run rviz and add an odometry display on the /odom topic, with the fixed frame set to /odom, and you'll see a pretty pattern
