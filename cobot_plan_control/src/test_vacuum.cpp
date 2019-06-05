#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>

#include "cobot_plan_control/tcp_client.h"

std::string str[] = {"-", "+"};
tcpClient vacuum;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_vacuum");

  try {
    ros::NodeHandle n;
    std::string port;
    std::ifstream file ("/home/mtec/catkin_ws/src/cobot/cobot_plan_control/cobot_server_setting.txt");
    if( file.is_open() )
    {
      if( getline(file, port) )
        ROS_INFO("/port : %s", port.c_str());
      file.close();
    }
    else
      exit(-1);

    std::string::size_type sz;   // alias of size_t
    int i_dec = std::stoi (port, &sz);
    vacuum.init(i_dec);
    vacuum.tcpWrite(str[0]);

    int i=0;
    ros::Rate loop_rate(2);
    while (ros::ok()) {
      vacuum.tcpWrite(str[i]);
      ROS_INFO("Operation : %s", str[i].c_str());
      loop_rate.sleep();
      i = (i+1)%2;
    }
  }
  catch(const std::string &err) {
    ROS_ERROR("%s", err.c_str());
  }

  ROS_INFO("end");
  ros::shutdown();
  return 0;
}
