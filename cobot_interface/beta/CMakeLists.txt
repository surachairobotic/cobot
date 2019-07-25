set(HEADERS
  include/cobot_interface_beta_display.h
  include/cobot_interface_beta_widget.h
)

qt_wrap_ui(UIC_FILES src/ui/cobot_interface_beta.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES
  src/cobot_interface_beta_display.cpp
  src/cobot_interface_beta_widget.cpp
)

set(COBOT_UI_LIB_NAME cobot_interface_beta)
add_library(${COBOT_UI_LIB_NAME}_core ${SOURCE_FILES} ${HEADERS} ${UIC_FILES})
set_target_properties(${COBOT_UI_LIB_NAME}_core PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${COBOT_UI_LIB_NAME}_core
  ${catkin_LIBRARIES}
  ${rviz_DEFAULT_PLUGIN_LIBRARIES}
  ${OGRE_LIBRARIES} ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
#  ${cobot_msgs_EXPORTED_TARGETS}
)

add_library(${COBOT_UI_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${COBOT_UI_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${COBOT_UI_LIB_NAME} ${COBOT_UI_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${COBOT_UI_LIB_NAME} ${COBOT_UI_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
