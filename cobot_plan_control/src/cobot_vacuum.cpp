#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/GetPositionFK.h>
#include <string>
#include <iostream>
#include <fstream>

#include "cobot_plan_control/tcp_client.h"

geometry_msgs::PoseArray p;
sensor_msgs::JointState js;
std::string str[] = {"-", "+"};
tcpClient vacuum;

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
    ros::Subscriber sub_execute = n.subscribe("/cobot/message", 1000, execute_callback);
    ros::Subscriber sub_js = n.subscribe("/joint_states", 1000, js_callback);

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

    ros::service::waitForService("/compute_fk");
    ros::ServiceClient fk_client = n.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
    moveit_msgs::GetPositionFK::Request  fk_request;
    moveit_msgs::GetPositionFK::Response fk_response;
    fk_request.fk_link_names = {"tool0"};

    ros::Rate loop_rate(10);
    while (ros::ok()) {
      fk_request.header = js.header;
      fk_request.robot_state.joint_state = js;

      if(p.poses.size() > 2) {
        if(fk_client.call(fk_request, fk_response))
        {
          if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
          {
            for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
            {
              if(fk_response.fk_link_names[i].find("tool0") != -1) {
                double dist = distance_point(p.poses[1], fk_response.pose_stamped[i].pose);
                if(dist <= 0.02)
                  vacuum.tcpWrite(str[1]);
                if(p.poses.size() == 6) {
                  if(distance_point(p.poses[4], fk_response.pose_stamped[i].pose) <= 0.02)
                    vacuum.tcpWrite(str[0]);
                }
                break;
              }
            }
          }
          else
          {
            ROS_ERROR("Forward kinematics failed");
          }
        }
        else
        {
          ROS_ERROR("Forward kinematics service call failed");
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  catch(const std::string &err) {
    ROS_ERROR("%s", err.c_str());
  }

  ROS_INFO("end");
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
  if(exe.data)
    vacuum.tcpWrite(str[0]);
}

double distance_point(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2) {
  return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x)
	    + (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y)
	    + (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
}
