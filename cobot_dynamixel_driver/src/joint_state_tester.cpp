#include "cobot_dynamixel_driver/common.h"

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

sensor_msgs::JointState jntReturn;
bool b_return, b_lock;

ros::Publisher pub_return;

void control_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  ROS_WARN("void control_callback : position %lf", msg->position[0]);
/*  b_return = false;
  if(!b_lock) {
    jntReturn.name = msg->name;
    jntReturn.position = msg->position;
    jntReturn.velocity = msg->velocity;
    jntReturn.effort = msg->effort;
  }
  b_return = true;
*/
  pub_return.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tester");
  ros::NodeHandle n;
  pub_return = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/joint_states_return", 1000);

  ros::Subscriber sub = n.subscribe("cobot_dynamixel_driver/goal", 1000, control_callback);
  //ros::Rate loop_rate(100);

  ros::spin();
  return 0;
}
