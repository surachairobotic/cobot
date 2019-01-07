cmake_minimum_required(VERSION 2.8.3)
project(cobot_jig_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
#add_compile_options(-std=c++11 -g -O0)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
  rospy
  std_msgs
  cobot_sample_controller
  cobot_dynamixel_driver
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    system_lib
    EIGEN3
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  include
)

add_library(traj_controller_class
  src/traj_controller.cpp
	src/MathFunctions.cpp
)
target_link_libraries(traj_controller_class
											${catkin_LIBRARIES}
											${Boost_LIBRARIES})

#add_executable(cobot_jig_controller src/jig_controller.cpp)
#add_executable(cobot_jig_controller src/my_controller.cpp)
#add_executable(cobot_jig_controller src/velocity_controller.cpp)
add_executable(cobot_jig_controller src/real_velocity_controller.cpp)
#add_executable(cobot_jig_controller src/pi_control.cpp)

target_link_libraries(cobot_jig_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  cobot_controller_class
	traj_controller_class
)

add_executable(cobot_pi_vel src/setup_pi_vel.cpp)
target_link_libraries(cobot_pi_vel
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  cobot_controller_class
)

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   # myfile1
   # myfile2
   up_one.srv
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

