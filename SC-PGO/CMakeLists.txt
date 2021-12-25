cmake_minimum_required(VERSION 2.8.3)
project(livox_mapping)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  ad_localization_msgs
  image_transport
  cv_bridge
  tf
  tf_conversions
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs ad_localization_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)
add_executable(livox_mapping 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
target_compile_options(livox_mapping 
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(livox_mapping 
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)





