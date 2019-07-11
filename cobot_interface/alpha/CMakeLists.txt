set(HEADERS_ALPHA
  include/cobot_display.h
  include/cobot_widget.h
  include/cobot_label.h
)

qt_wrap_ui(UIC_FILES_ALPHA src/ui/cobot_interface_alpha.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES_ALPHA
  src/cobot_display.cpp
  src/cobot_widget.cpp
  src/cobot_widget_teach.cpp
  src/cobot_widget_jog.cpp
  src/cobot_label.cpp
)

set(COBOT_UI_LIB_NAME_ALPHA cobot_interface_alpha)
add_library(${COBOT_UI_LIB_NAME_ALPHA}_core ${SOURCE_FILES_ALPHA} ${HEADERS_ALPHA} ${UIC_FILES_ALPHA})
set_target_properties(${COBOT_UI_LIB_NAME_ALPHA}_core PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${COBOT_UI_LIB_NAME_ALPHA}_core
  ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${COBOT_UI_LIB_NAME_ALPHA} src/plugin_init.cpp)
set_target_properties(${COBOT_UI_LIB_NAME_ALPHA} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${COBOT_UI_LIB_NAME_ALPHA} ${COBOT_UI_LIB_NAME_ALPHA}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${COBOT_UI_LIB_NAME_ALPHA} ${COBOT_UI_LIB_NAME_ALPHA}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
