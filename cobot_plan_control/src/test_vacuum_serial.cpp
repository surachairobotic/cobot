#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <algorithm>

#include "cobot_plan_control/cSerial.h"

using namespace std;
using namespace boost::filesystem;

std::string str[] = {"$A0#", "$A1#"};
cSerial *vacuum;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_vacuum");

  try {
    ros::NodeHandle n;
    std::string port;
    try {
      path p("/dev");
      vector<std::string> port;
      for (auto i = directory_iterator(p); i != directory_iterator(); i++)
      {
          if (!is_directory(i->path())) //we eliminate directories
          {
            if ( i->path().filename().string().find("ACM") != std::string::npos )
              port.push_back( "/dev/" + i->path().filename().string() );
          }
          else
            continue;
      }
      if( port.size() == 0 )
        throw std::string("Can't find USB port");
      for( int i=0; i<port.size(); i++) {
        ROS_INFO("%s", port[i].c_str());
        vacuum = new cSerial();
        ros::Time t_start = ros::Time::now();
        ROS_INFO("start");
        if( !vacuum->open(port[i].c_str(), 9600) ) {
          throw std::string("Can't open port : %s", port[i].c_str());
        }
        char buff[128];
        int len = 0;
        std::string msgs = "$id#";
        bool chk = true;
        double dt = 0.0;
        int cnt = 0;
        do {
          ROS_INFO("cnt=%d", cnt++);
          vacuum->write(msgs.c_str());
          int len = vacuum->read(buff, sizeof(buff));
          if( len > 0 ) {
            if ( buff[0] == '$' && buff[3] == '#' && buff[1] == '0' && buff[2] == '1' )
              chk = false;
            else
              ROS_INFO("buff(%d)= %s", len, buff);
          }
          dt = (ros::Time::now()-t_start).toSec();
        } while(ros::ok() && chk && dt<1.0);
        ROS_INFO("dt:%lf", dt);
      }
    }
    catch(const std::runtime_error& re)
    {
        // speciffic handling for runtime_error
        std::cerr << "Runtime error: " << re.what() << std::endl;
    }
    catch(const std::exception& ex)
    {
        // speciffic handling for all exceptions extending std::exception, except
        // std::runtime_error which is handled explicitly
        std::cerr << "Error occurred: " << ex.what() << std::endl;
    }
    catch(const std::string &err){
        ROS_ERROR("%s", err.c_str());
    }
    catch(...)
    {
        // catch any other errors (that we have no information about)
        std::cerr << "Unknown failure occurred. Possible memory corruption" << std::endl;
    }
    ROS_INFO("port : %s", vacuum->port_name);

    int i=0;
    ros::Rate loop_rate(2);
    while (ros::ok()) {
      vacuum->write(str[i].c_str());
      ROS_DEBUG("Operation : %s", str[i].c_str());
      ROS_INFO("Operation : %s", str[i].c_str());
      ROS_WARN("Operation : %s", str[i].c_str());
      ROS_ERROR("Operation : %s", str[i].c_str());
      ROS_FATAL("Operation : %s", str[i].c_str());
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
