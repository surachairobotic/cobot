#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/GetPositionFK.h>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <algorithm>

#include "cobot_plan_control/cSerial.h"

using namespace std;
using namespace boost::filesystem;

geometry_msgs::PoseArray p;
sensor_msgs::JointState js;
std::string str[] = {"$A0#", "$A1#"};
cSerial *vacuum;

void pose_callback(const geometry_msgs::PoseArray& msg);
void js_callback(const sensor_msgs::JointState& msg);
void execute_callback(const std_msgs::Bool exe);
double distance_point(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2);
void get_cartesian_position(const std::vector<double> &joint_pos, geometry_msgs::Pose &pose);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_vacuum");

  try {
    ros::NodeHandle n;
    ros::Subscriber sub_pose = n.subscribe("/cobot/pose_to_control", 1000, pose_callback);
    ros::Subscriber sub_execute = n.subscribe("/cobot/vacuum_message", 10, execute_callback);
    ros::Subscriber sub_js = n.subscribe("/joint_states", 1000, js_callback);

    errno = 0;
    ROS_DEBUG("first len err : %d", errno);
    bool find = false;
    int baud_rate = 9600;
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
        ROS_DEBUG("%s", port[i].c_str());
        vacuum = new cSerial();
        ros::Time t_start = ros::Time::now();

        char *cstr = new char[port[i].length() + 1];
        strcpy(cstr, port[i].c_str());

        if( !vacuum->open(cstr, baud_rate) ) {
          throw std::string("Can't open port : %s", port[i].c_str());
        }
        char buff[128];
        std::string msgs = "$id#";
        double dt = 0.0;
        do {
          int len = vacuum->write(msgs.c_str());
          if( len!=msgs.size() ){
            ROS_DEBUG("write error : len = %d", len);
          }
          usleep(200000);
          ROS_DEBUG("send : %s", msgs.c_str());
          len = vacuum->read(buff, sizeof(buff));
          if( len<0 ){
            ROS_DEBUG("len err : %d", errno);
          }
          if( len > 0 ) {
            if ( buff[0] == '$' && buff[3] == '#' && buff[1] == '0' && buff[2] == '1' ){
              ROS_DEBUG("found");
              find = true;
            }
            else{
              buff[sizeof(buff)-1] = 0;
              ROS_DEBUG("buff(%d)= %s", len, buff);
            }
          }
          dt = (ros::Time::now()-t_start).toSec();
        } while(ros::ok() && !find && dt<10.0);
        ROS_DEBUG("dt:%lf", dt);
        // do stuff
        delete [] cstr;
      }
    }
    catch(const std::runtime_error& re) {
        // speciffic handling for runtime_error
        std::cerr << "Runtime error: " << re.what() << std::endl;
        exit(0);
    }
    catch(const std::exception& ex) {
        // speciffic handling for all exceptions extending std::exception, except
        // std::runtime_error which is handled explicitly
        std::cerr << "Error occurred: " << ex.what() << std::endl;
        exit(0);
    }
    catch(const std::string &err) {
        ROS_ERROR("%s", err.c_str());
        exit(0);
    }
    catch(...) {
        // catch any other errors (that we have no information about)
        std::cerr << "Unknown failure occurred. Possible memory corruption" << std::endl;
        exit(0);
    }
    if (!find) {
      ROS_ERROR("Can't open port : timeout");
      exit(0);
    }
    else
      ROS_DEBUG("port : %s", vacuum->port_name);

    // vacuum.init(i_dec);
    // vacuum.tcpWrite(str[0]);

    ros::service::waitForService("/compute_fk");
    // ros::ServiceClient fk_client = n.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
    moveit_msgs::GetPositionFK::Request  fk_request;
    moveit_msgs::GetPositionFK::Response fk_response;
    fk_request.fk_link_names = {"tool0"};

    ROS_INFO("Ready : Cobot Vacuum");
    ros::Rate loop_rate(100);
    while (ros::ok()) {
      // fk_request.header = js.header;
      // fk_request.robot_state.joint_state = js;
      //
      // if(p.poses.size() > 2) {
      //   if(fk_client.call(fk_request, fk_response))
      //   {
      //     if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
      //     {
      //       for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
      //       {
      //         if(fk_response.fk_link_names[i].find("tool0") != -1) {
      //           double dist = distance_point(p.poses[1], fk_response.pose_stamped[i].pose);
      //           ROS_DEBUG("dist : %lf", dist);
      //           if(dist <= 0.05)
      //             vacuum->write(str[1].c_str());
      //           if(p.poses.size() == 6) {
      //             if(distance_point(p.poses[4], fk_response.pose_stamped[i].pose) <= 0.05)
      //               vacuum->write(str[0].c_str());
      //           }
      //           break;
      //         }
      //       }
      //     }
      //     else
      //     {
      //       ROS_ERROR("Forward kinematics failed");
      //     }
      //   }
      //   else
      //   {
      //     ROS_ERROR("Forward kinematics service call failed");
      //   }
      // }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  catch(const std::string &err) {
    ROS_ERROR("%s", err.c_str());
  }

  delete vacuum;
  ROS_DEBUG("end");
  ros::shutdown();
  return 0;
}

void pose_callback(const geometry_msgs::PoseArray& msg) {
  p = msg;
}

void js_callback(const sensor_msgs::JointState& msg) {
  js = msg;
}

void execute_callback(const std_msgs::Bool exe) {
  ROS_INFO("execute_callback : %d", exe.data);
  if(exe.data)
    vacuum->write(str[1].c_str());
  else
    vacuum->write(str[0].c_str());
}

double distance_point(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2) {
  return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x)
	    + (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y)
	    + (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
}
