cmake_minimum_required(VERSION 2.8.12)
project(my_ui)

add_compile_options(-std=c++11)

# definition needed for boost/math/constants/constants.hpp included by Ogre to compile
add_definitions(-DBOOST_MATH_DISABLE_FLOAT128)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread date_time system filesystem)

find_package(PkgConfig REQUIRED)

# Ogre
pkg_check_modules(OGRE OGRE)
link_directories( ${OGRE_LIBRARY_DIRS} )

find_package(catkin REQUIRED COMPONENTS
#  moveit_ros_planning_interface
  moveit_ros_robot_interaction
#  moveit_ros_warehouse
#  moveit_ros_perception
  eigen_conversions
  geometric_shapes
  interactive_markers
  class_loader
  rviz
  tf
  roscpp
  rosconsole
#  object_recognition_msgs
  rostest

    moveit_core
#    moveit_ros_planning
    pluginlib
    moveit_visual_tools
)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
		my_ui_01_core
  INCLUDE_DIRS
		my_ui_01/include
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    moveit_ros_robot_interaction
    moveit_core
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(my_ui_01/include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS}
										${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(my_ui_01)

install(FILES
	my_ui_01.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
