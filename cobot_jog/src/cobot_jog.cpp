#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "cobot_msgs/Jog.h"

void callback_js(const sensor_msgs::JointState &js);
void callback_jog(const cobot_msgs::Jog &msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "cobot_jog");
  ros::NodeHandle n;
  ros::Subscriber sub_js = n.subscribe("/cobot/joint_states", 100, callback_js);
  ros::Subscriber sub_ui = n.subscribe("/cobot/cobot_jog/jog", 100, callback_jog);

  ROS_INFO("COBOT_JOG : START");
  ros::Rate loop_rate(100);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("COBOT_JOG : END");
}

void callback_js(const sensor_msgs::JointState &_js) {
  ;//js = _js;
}

void callback_jog(const cobot_msgs::Jog &msg) {
  ROS_INFO("callback_jog");
}
