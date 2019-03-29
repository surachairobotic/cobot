#include "ros/ros.h"
#include "cobot_sample_controller/cControl.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include <numeric>

#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
//using namespace std;

#include <cstddef>

#include "cobot_plan_control/tcp_client.h"

template <typename Container>
Container& split(
  Container&                            result,
  const typename Container::value_type& s,
  const typename Container::value_type& delimiters,
  int                                   empties = 0 )
{
  result.clear();
  size_t current;
  size_t next = -1;
  do
  {
    if (empties == 1)
    {
      next = s.find_first_not_of( delimiters, next + 1 );
      if (next == Container::value_type::npos) break;
      next -= 1;
    }
    current = next + 1;
    next = s.find_first_of( delimiters, current );
    result.push_back( s.substr( current, next - current ) );
  }
  while (next != Container::value_type::npos);
  return result;
}

tcpClient client;
cControl *p_control = NULL;
ros::Publisher pub_plan;
ros::Publisher pub_message;

void pose_callback(const geometry_msgs::PoseArray& msg);
void execute_callback(const std_msgs::Bool exe);
const moveit_msgs::RobotTrajectory& plan_line_two_pose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, float step);
bool move_trajectory(cControl &control);
double distance_point(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2);
void print_move_cartesian(FILE *fp, double t_wp, const geometry_msgs::Point &wp, const std::vector<double> &wp_velo, double t_mv, const geometry_msgs::Point &mv, const std::vector<double> &mv_velo);
void print_move_joint_1(FILE *fp, double t, const std::vector<double> &p_wp, const std::vector<double> &v_wp, const std::vector<double> &p_jnt, const std::vector<double> &v_jnt);
void print_move_joint_2(FILE *fp, const std::vector<double> &v_cal);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_plan_control");
  cControl control("arm", "tool0");
  p_control = &control;
  
  ros::Rate loop_rate(10);

  try {
    std::string dir = "/home/mtec/catkin_ws/src/cobot/cobot_jig_controller/results/";
    std::vector<std::string> lines;
    std::string line;
    std::ifstream myfile (dir + "point_cloud_03.txt");
    std::vector<geometry_msgs::Point> points;
    if(myfile.is_open())
    {
      while( std::getline(myfile,line) )
      {
        std::vector<std::string> tmp;
        ROS_INFO("%s", line.c_str());
        split( tmp, line, ",", 1 );
        std::string::size_type sz;
        geometry_msgs::Point pnt;
        pnt.x = std::stof(tmp[0],&sz);
        pnt.y = std::stof(tmp[1],&sz);
        pnt.z = std::stof(tmp[2],&sz);
        points.push_back(pnt);
      }
      ROS_INFO("File is ok.");
      myfile.close();
    }
    else {
      ROS_ERROR("Unable to open file");
      geometry_msgs::Point pnt;
      pnt.x = 0.0;
      pnt.y = 0.0;
      pnt.z = 0.0;
      points.push_back(pnt);
//      return -1;
    }

    client.init();
    std::string str = "-";
    if(client.tcpWrite(str))
      ROS_INFO("tcpWrite is true");
    else
      ROS_INFO("tcpWrite is false");
    control.init();
    ros::NodeHandle n;
    ros::Subscriber sub_pose = n.subscribe("/cobot/pose", 1000, pose_callback);
    ros::Subscriber sub_execute = n.subscribe("/cobot/execute", 1000, execute_callback);
    pub_plan = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher pub_polygon = n.advertise<visualization_msgs::Marker>("/cobot/marker_polygon", 100);
    ros::Publisher pub_lines = n.advertise<visualization_msgs::Marker>("/cobot/marker_lines", 100);
    pub_message = n.advertise<std_msgs::Bool>("/cobot/message", 100);
    
    sensor_msgs::JointState tmp_jnt;
    ROS_INFO("start...");
    while (ros::ok()) {
/*      if( control.wait_new_joint_state(&tmp_jnt, 1.0) )
        ROS_INFO("Joint_state is OK");
      else
        ROS_ERROR("Joint_state is ERROR");
*/    
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "polygon_shapes";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      
      marker.points = points;
      marker.lifetime = ros::Duration();
    
      pub_polygon.publish(marker);    

      marker.ns = "lines_shapes";
      marker.id = 1;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;

      marker.scale.x = 0.001;
      marker.scale.y = 0.001;
      marker.scale.z = 0.001;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5;

      marker.points.clear();
      for(int i=0; i<points.size(); i+=3) {
        marker.points.push_back(points[i]);
        marker.points.push_back(points[i+1]);
        marker.points.push_back(points[i]);
        marker.points.push_back(points[i+2]);
        marker.points.push_back(points[i+1]);
        marker.points.push_back(points[i+2]);
      }

      pub_lines.publish(marker);    
    
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

void execute_callback(const std_msgs::Bool exe)
{
  ROS_INFO("execute_callback");
  if(exe.data)
    move_trajectory(*p_control);
  std::string str = "+";
  if(client.tcpWrite(str))
    ROS_INFO("tcpWrite is true");
  else
    ROS_INFO("tcpWrite is false");
  std_msgs::Bool send;
  send.data = true;
  pub_message.publish(send);
}

void pose_callback(const geometry_msgs::PoseArray& msg) {
  ROS_INFO("pose_callback");
  moveit_msgs::DisplayTrajectory dis_traj;
//  p_control->replan_velocity( req.max_velocity, req.max_acceleration);
  dis_traj.trajectory.push_back(plan_line_two_pose(msg.poses[0], msg.poses[1], 0.01));
  pub_plan.publish(dis_traj);
}

const moveit_msgs::RobotTrajectory& plan_line_two_pose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, float step) {
  p_control->plan_line(p1, p2, step);
  p_control->replan_velocity(0.1, 0.25);
  return p_control->get_robot_trajectory();
}

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// move_trajectory //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
bool move_trajectory(cControl &control) {
  printf("move_trajectory\n");
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  int i=0;
  trajectory_msgs::JointTrajectoryPoint p = traj.points[i];
  
  // set acc to 3.14 rad/sec^2
  std::vector<double> acc(6);
  for(int i=0;i<acc.size();i++)
    acc[i] = M_PI;
  control.set_acc(acc);

  sensor_msgs::JointState joint_state;
  if( !control.wait_new_joint_state(&joint_state, 1.0) ) return false; // exit program if the new state does not come

  geometry_msgs::Pose jnt_pose, wp_pose;
	std::vector<double> jnt_vel, wp_vel;
  control.get_cartesian_position(joint_state.position, jnt_pose);
  control.get_cartesian_velocity(joint_state.position, joint_state.velocity, jnt_vel);
  control.get_cartesian_position(p.positions, wp_pose);
  control.get_cartesian_velocity(p.positions, p.velocities, wp_vel);

  for(int i=0; i<6; i++)
    wp_vel[i] = 0.25;
  control.move_pos_velo(p.positions, wp_vel);
  ros::Time t = ros::Time::now();

  bool stop = false;
  while( !stop && ros::ok() ) {
    if( !control.wait_new_joint_state(&joint_state, 1.0) ) {
			goto LB_EXIT_MOVE; // exit program if the new state does not come
		}

    control.get_cartesian_position(joint_state.position, jnt_pose);
    control.move_pos_velo(p.positions, wp_vel);

    if( distance_point(jnt_pose, wp_pose) < 0.015 ) {
      if( i < traj.points.size()-1 ) {
        i++;
        p = traj.points[i];
        control.get_cartesian_position(p.positions, wp_pose);
//        control.get_cartesian_velocity(p.positions, p.velocities, wp_vel);
      }
      else
        stop = true;
    }
  }

LB_EXIT_MOVE:
	for(int j=0; j<p.velocities.size(); j++)
	  p.velocities[j] = 0.0;
	control.move_velo(p.velocities);
  printf("move_traj end\n");

	return true;
}

double distance_point(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2) {
  return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x)
	    + (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y)
	    + (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
}

void print_move_cartesian(FILE *fp, double t_wp, const geometry_msgs::Point &wp, const std::vector<double> &wp_velo, double t_mv, const geometry_msgs::Point &mv, const std::vector<double> &mv_velo)
{
	fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf"
						, t_wp, t_mv, wp.x, wp.y, wp.z, mv.x, mv.y, mv.z);
	for(int i=0; i<3; i++)
		fprintf(fp, " %lf", wp_velo[i]);
	for(int i=0; i<3; i++)
		fprintf(fp, " %lf", mv_velo[i]);
	fprintf(fp, "\n");
	return ;
}

void print_move_joint_1(FILE *fp, double t, const std::vector<double> &p_wp, const std::vector<double> &v_wp, const std::vector<double> &p_jnt, const std::vector<double> &v_jnt)
{
	fprintf(fp, "%lf", t);
	for(int i=0; i<p_wp.size(); i++)
		fprintf(fp, " %lf", p_wp[i]);
	for(int i=0; i<v_wp.size(); i++)
		fprintf(fp, " %lf", v_wp[i]);
	for(int i=0; i<p_jnt.size(); i++)
		fprintf(fp, " %lf", p_jnt[i]);
	for(int i=0; i<v_jnt.size(); i++)
		fprintf(fp, " %lf", v_jnt[i]);
	return ;
}

void print_move_joint_2(FILE *fp, const std::vector<double> &v_cal)
{
	for(int i=0; i<v_cal.size(); i++)
		fprintf(fp, " %lf", v_cal[i]);
	fprintf(fp, "\n");
	return ;
}

