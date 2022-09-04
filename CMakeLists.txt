cmake_minimum_required(VERSION 3.0.2)
project(se3_controller)
add_compile_options(-std=c++14)

set(airsim_ros_pkgs_DIR /home/ck1201/workspace/MAS/RMUAV/IntelligentUAVChampionshipSimulator/roswrapper/ros/devel/share/airsim_ros_pkgs/cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  airsim_ros_pkgs
)

find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES se3_controller
#  CATKIN_DEPENDS geometry_msgs nav_msgs quadrotor_msgs roscpp rospy std_msgs tf airsim_ros_pkgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(${PROJECT_NAME}_example_airsim_node 
  src/se3_example_airsim.cpp
)

target_link_libraries(${PROJECT_NAME}_example_airsim_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_example_node 
  src/se3_example.cpp
)

target_link_libraries(${PROJECT_NAME}_example_node
  ${catkin_LIBRARIES}
)