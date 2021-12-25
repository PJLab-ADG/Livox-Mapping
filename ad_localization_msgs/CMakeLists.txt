cmake_minimum_required(VERSION 3.0.2)
project(ad_localization_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs geometry_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  NavStateInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)