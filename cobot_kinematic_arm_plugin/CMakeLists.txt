cmake_minimum_required(VERSION 2.8.12)
project(cobot_kinematic_arm_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(COBOT_KINEMATIC_LIBRARY_NAME cobot_kinematic_moveit_plugin)

find_package(LAPACK REQUIRED)

add_library(${COBOT_KINEMATIC_LIBRARY_NAME} src/cobot_kinematic_moveit_plugin.cpp)
target_link_libraries(${COBOT_KINEMATIC_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${COBOT_KINEMATIC_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  cobot_kinematic_moveit_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
