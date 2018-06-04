cmake_minimum_required(VERSION 2.8.3)
project(cobot_planner)

add_compile_options(-std=c++11 -g -O0)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(Boost REQUIRED system filesystem date_time thread)


## msg, stv ###


add_service_files(
  FILES
  CobotPlanning.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime
    roscpp
    rospy
    std_msgs
    geometry_msgs
    trajectory_msgs
    moveit_core
    moveit_ros_planning_interface
  DEPENDS
    system_lib
)



###########
## Build ##
###########



include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  include
)


add_executable(cobot_planner src/main.cpp src/cControl.cpp)
add_dependencies(cobot_planner cobot_planner_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(cobot_planner
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############



