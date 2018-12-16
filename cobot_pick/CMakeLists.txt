cmake_minimum_required(VERSION 2.8.3)
project(cobot_pick)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11 -g -O0)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  image_geometry
  visualization_msgs
  geometry_msgs

  sensor_msgs
  cv_bridge
  image_transport
)

find_package(PCL REQUIRED)
# find_package( TESSERACT REQUIRED )
find_package(TBB COMPONENTS tbbmalloc tbbmalloc_proxy tbb_preview)
add_definitions(${PCL_DEFINITIONS})

find_package( PkgConfig REQUIRED)
pkg_search_module( TESSERACT REQUIRED tesseract )
pkg_search_module( LEPTONICA REQUIRED lept )
include_directories( ${TESSERACT_INCLUDE_DIRS} )
include_directories( ${LEPTONICA_INCLUDE_DIRS} )
link_directories( ${TESSERACT_LIBRARY_DIRS} )
link_directories( ${LEPTONICA_LIBRARY_DIRS} )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cobot_pick
#  CATKIN_DEPENDS opencv2 roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#   ${TESSERACT_INCLUDE_DIRS}
)


link_directories(/usr/local/lib ${PCL_LIBRARY_DIRS})
add_executable(cobot_find_object src/cobot_find_object.cpp src/cLabeling.cpp src/zdsfmt.cpp)
target_link_libraries(
  cobot_find_object
  ${catkin_LIBRARIES}
  opencv_core
  opencv_imgcodecs
  opencv_highgui
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  tbb
  ${TESSERACT_LIBRARIES}
  ${LEPTONICA_LIBRARIES}
)

