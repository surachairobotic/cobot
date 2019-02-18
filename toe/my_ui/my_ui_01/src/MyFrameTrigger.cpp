#include <MyFrame.h>
#include <MyDisplay.h>
#include "std_msgs/Bool.h"

#include "ui_multimovedisplay.h"

using namespace my_plugin;

void MyFrame::triggerClicked()
{
  std_msgs::Bool command;
  command.data = true;
  trigger_pub.publish(command);
  ROS_ERROR("end_link : %s", planning_display_->end_link.c_str());
}

