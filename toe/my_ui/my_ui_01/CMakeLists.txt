set(HEADERS
	include/MyFrame.h
	include/MyDisplay.h

	include/pose_string.h
	include/imarker.h
	include/interactive_robot.h

	include/cobot_kinematic.h
)

qt_wrap_ui(UIC_FILES src/ui/multimovedisplay.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES
	src/MyFrame.cpp
	src/MyFrameIK.cpp
	src/MyFrameMove.cpp
	src/MyDisplay.cpp
	src/pose_string.cpp
	src/imarker.cpp
	src/interactive_robot.cpp
	src/cobot_kinematic.cpp
)

set(MOVEIT_LIB_NAME my_ui_01)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${HEADERS} ${UIC_FILES})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME}_core
#  moveit_rviz_plugin_render_tools
#  moveit_planning_scene_rviz_plugin_core
  ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
