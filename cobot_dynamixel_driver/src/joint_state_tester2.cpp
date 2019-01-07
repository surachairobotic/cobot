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

void control_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  ROS_WARN("recv : position %lf", msg->position[0]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tester2");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/goal", 1000);
  ros::Subscriber sub = n.subscribe("cobot_dynamixel_driver/joint_states_return", 1000, control_callback);

  sensor_msgs::JointState jnt;
  ROS_INFO("send : cobot_dynamixel_driver/goal");

  ros::Rate loop_rate(100);
  jnt.position.resize(1);
  int i=0;
  while(1) {
    ROS_INFO("i = %d", i);
    jnt.position[0] = i++;
    pub.publish(jnt);
    for(int j=0;j<10;j++){
      ros::spinOnce();
      loop_rate.sleep();
    }
    if(i>=20) break;
  }
  
  ros::spinOnce();
  return 0;
}
