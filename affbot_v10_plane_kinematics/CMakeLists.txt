cmake_minimum_required(VERSION 2.8.3)
project(affbot_v10_plane_kinematics)

add_compile_options(-std=c++11 -g -O0)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  rosconsole
  urdf
  kdl_parser
  tf
  tf_conversions
  eigen_conversions
  control_msgs
  moveit_msgs
#  pr2_mechanism_msgs
#  pr2_controllers_msgs
  cmake_modules
  REQUIRED)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
)

#include_directories(pr2_arm_kinematics/include)    


include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

###########
## Build ##
###########


set(MOVEIT_LIB_NAME affbot_v10_plane_kinematics)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_library(${MOVEIT_LIB_NAME} 
  src/affbot_ik.cpp
  src/affbot_ik_solver.cpp
  src/affbot_kinematics_utils.cpp
  src/affbot_kinematics_plugin.cpp
  )


target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(
  FILES
    affbot_v10_plane_kinematics_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )


