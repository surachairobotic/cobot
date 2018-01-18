https://github.com/ros-planning/moveit_pr2/blob/groovy-devel/pr2_moveit_plugins/pr2_arm_kinematics/src/main.cpp

#include <affbot_kinematics/affbot_kinematics.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "affbot_kinematics");
  affbot_kinematics::AffbotKinematics affbot_kinematics;

  if(!affbot_kinematics.isActive())
  {
    ROS_ERROR("affbot_kinematics could not be activated");
  }
  else
  {
    ROS_INFO("affbot_kinematics active");
    ros::spin();
  }
  return(0);
}
