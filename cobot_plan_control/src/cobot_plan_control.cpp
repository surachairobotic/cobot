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

cControl *p_control = NULL;
ros::Publisher pub_plan;

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

    control.init();
    ros::NodeHandle n;
    ros::Subscriber sub_pose = n.subscribe("/cobot/pose", 1000, pose_callback);
    ros::Subscriber sub_execute = n.subscribe("/cobot/execute", 1000, execute_callback);
    pub_plan = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher pub_polygon = n.advertise<visualization_msgs::Marker>("/cobot/marker_polygon", 100);
    ros::Publisher pub_lines = n.advertise<visualization_msgs::Marker>("/cobot/marker_lines", 100);
    
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
}

void pose_callback(const geometry_msgs::PoseArray& msg) {
  ROS_INFO("pose_callback");
  moveit_msgs::DisplayTrajectory dis_traj;
//  p_control->replan_velocity( req.max_velocity, req.max_acceleration);
  dis_traj.trajectory.push_back(plan_line_two_pose(msg.poses[0], msg.poses[1], 0.01));
//  pub_plan.publish(dis_traj);
}

const moveit_msgs::RobotTrajectory& plan_line_two_pose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, float step) {
  p_control->plan_line(p1, p2, step);
//  p_control->replan_velocity(0.1, 0.3);
  return p_control->get_robot_trajectory();
}

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// move_trajectory //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
bool move_trajectory(cControl &control) {
  printf("move_trajectory\n");
  std::string result_dir = "/home/mtec/catkin_ws/src/cobot/cobot_jig_controller/results/";
  FILE *fp1 = fopen( (result_dir + "move_cartesian_indv.txt").c_str() , "wt");
  FILE *fp2 = fopen( (result_dir + "move_joint_indv.txt").c_str() , "wt");
  ROS_ERROR("result_dir : %s", result_dir.c_str());
  ROS_ERROR("*fp1 : %s", (result_dir + "move_cartesian_indv.txt").c_str());
  ROS_ERROR("*fp2 : %s", (result_dir + "move_joint_indv.txt").c_str());
  if(fp1 == NULL)
    ROS_ERROR("fp1 is NULL");
  if(fp2 == NULL)
    ROS_ERROR("fp2 is NULL");
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();

  char target_str[512];

  ros::Rate r(100);
  ros::Time t_start = ros::Time::now(), t_print = t_start, start_time, read_time, run_time;
  
  // set acc to 3.14 rad/sec^2
  std::vector<double> acc(6);
  for(int i=0;i<acc.size();i++)
    acc[i] = M_PI;
  control.set_acc(acc);

	bool first_time = true;
  sensor_msgs::JointState joint_state, j_state;
  int i = 1;
  trajectory_msgs::JointTrajectoryPoint p = traj.points[i];
  geometry_msgs::Pose wp_pose, current_pose, ex_pose, pose, tmp_pose;
	std::vector<double> wp_velo, mv_velo, wp_j_velo;
  ros::Time t = ros::Time::now();

	wp_j_velo.clear();
	for(int i=0; i<p.velocities.size(); i++)
		wp_j_velo.push_back(p.velocities[i]);

  if( !control.wait_new_joint_state(&joint_state, 1.0) ) return false; // exit program if the new state does not come
  control.get_cartesian_position(joint_state.position, current_pose);
  control.get_cartesian_position(p.positions, wp_pose);
  control.get_cartesian_velocity(p.positions, p.velocities, wp_velo);
  control.get_cartesian_velocity(joint_state.position, joint_state.velocity, mv_velo);
  std::vector<double> current_joint = control.get_current_joints();

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Loop  ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
  bool stop = false, b_jnt_state = false;
	double dis_min, dt, limit_vel = M_PI / 2.0, sum, d_lab, d_time_move;
	int j, k;
	std::vector<double> old_err(6, 0.0);
	std::vector<std::vector<double>> vec_err_v(6);
	std::vector<double> kp, kpp, ki;
	kp.push_back(0.7);
	kp.push_back(0.7);
	kp.push_back(0.5);
	kp.push_back(0.5);
	kp.push_back(0.3);
	kp.push_back(0.5);
	kpp.push_back(1.0);
	kpp.push_back(0.3);
	kpp.push_back(0.1);
	kpp.push_back(0.1);
	kpp.push_back(1.0);
	kpp.push_back(0.1);
	ki.push_back(0.01);
	ki.push_back(0.05);
	ki.push_back(0.05);
	ki.push_back(0.005);
	ki.push_back(0.05);
	ki.push_back(0.05);
  while( !stop && ros::ok() ) {
    if( !control.wait_new_joint_state(&joint_state, 1.0) ) {
			b_jnt_state = true;
			goto LB_EXIT_MOVE; // exit program if the new state does not come
		}

		run_time = read_time;
		read_time = ros::Time::now();
		if(first_time) start_time = read_time;
		d_lab = (read_time-run_time).toSec()*1000;
		d_time_move = (read_time-start_time).toSec();
		if(d_lab >= 42.5)
			ROS_WARN("[%.2lf s.][%.2lf s.] : %.2lf ms.", p.time_from_start.toSec(), d_time_move, d_lab);
		else
			ROS_INFO("[%.2lf s.][%.2lf s.] : %.2lf ms.", p.time_from_start.toSec(), d_time_move, d_lab);

    control.get_cartesian_position(joint_state.position, current_pose);
	  control.get_cartesian_velocity(joint_state.position, joint_state.velocity, mv_velo);
		print_move_cartesian(fp1, p.time_from_start.toSec(), wp_pose.position, wp_velo, d_time_move, current_pose.position, mv_velo);

		dis_min = 999.999;
		dt = p.time_from_start.toSec() - d_time_move;
		//ROS_INFO("dt, p.time, read-start | %lf, %lf, %lf", dt, p.time_from_start.toSec(), (read_time - start_time).toSec());
		if(dt < 0.02) {
			ROS_WARN("dt : %lf", dt);
			dt = 0.02;
		}

		for(int i=0; i<p.accelerations.size(); i++)
		  acc[i] = ( p.velocities[i]-joint_state.velocity[i] ) / dt;
//		control.set_acc(acc);

    std::vector<double> vel(6, 0.0);
    for(int i=0; i<p.velocities.size(); i++) {
      double target = p.velocities[i];
      double err_v = target - joint_state.velocity[i];
      double p_target = p.positions[i];
      double err_p = p_target-joint_state.position[i];
      vec_err_v[i].push_back( (old_err[i]+err_v)*dt/2.0 );
      while( vec_err_v[i].size() > 10 )
        vec_err_v[i].pop_back();

      double sum_of_elems = std::accumulate(vec_err_v[i].begin(), vec_err_v[i].end(), 0.0);
      vel[i] = target + (kp[i]*err_v) + (kpp[i]*err_p) + (ki[i]*sum_of_elems);
      old_err[i] = err_v;
    }

/*		for(int i=0; i<p.velocities.size(); i++) {
			if( p.positions[i] > joint_state.position[i] )
				p.velocities[i] = p.velocities[i] >= 0.0 ? p.velocities[i] : -p.velocities[i];
			else
				p.velocities[i] = p.velocities[i] >= 0.0 ? -p.velocities[i] : p.velocities[i];
		}
*/
/*		for(int i=0; i<p.positions.size(); i++) {
			ROS_INFO("distance[%d] : %lf", i, fabs(p.positions[i]-joint_state.position[i]) );
			if( fabs(p.positions[i]-joint_state.position[i]) <= 0.001 )
				p.velocities[i] = 0.0;
		}
*/
		print_move_joint_1(fp2, d_time_move, p.positions, wp_j_velo, joint_state.position, joint_state.velocity);

 	print_move_joint_2(fp2, p.velocities);

/*		for(int i=0; i<p.velocities.size(); i++)
			p.velocities[i] = 0.0;
		p.velocities[p.velocities.size()-1] = 0.5;
		for(int i=0; i<p.accelerations.size(); i++)
			p.accelerations[i] = M_PI;

		ROS_INFO("vel : %lf, %lf, %lf, %lf, %lf, %lf", p.velocities[0], p.velocities[1], p.velocities[2], p.velocities[3], p.velocities[4], p.velocities[5]);
		ROS_INFO("acc : %lf, %lf, %lf, %lf, %lf, %lf", p.accelerations[0], p.accelerations[1], p.accelerations[2], p.accelerations[3], p.accelerations[4], p.accelerations[5]);
*/
    control.move_velo_acc(vel, acc);
//    control.move_velo_acc(p.velocities, acc);
//    control.move_velo(p.velocities);

/*		sum = 0.0;
		do {
		  if( !control.wait_new_joint_state(&j_state, 1.0) ) {
				b_jnt_state = true;
				goto LB_EXIT_MOVE; // exit program if the new state does not come
			}
			for(k=0; k<joint_state.position.size(); k++)
				sum += fabs( joint_state.position[k] - j_state.position[k] );
		} while(sum < 0.0001);
*/
		if(first_time) {
			start_time = ros::Time::now();
			first_time = false;
		}

		ROS_INFO("xyz[%.2lf %.2lf %.2lf]:[%.2lf %.2lf %.2lf]", wp_pose.position.x, wp_pose.position.y, wp_pose.position.z, current_pose.position.x, current_pose.position.y, current_pose.position.z);
		ROS_INFO("dist : %lf", distance_point(current_pose, wp_pose));
    if( distance_point(current_pose, wp_pose) < 0.015 ) {
      if( i < traj.points.size()-1 ) {
        i++;
        p = traj.points[i];
				wp_j_velo.clear();
				for(int i=0; i<p.velocities.size(); i++)
					wp_j_velo.push_back(p.velocities[i]);
        control.get_cartesian_position(p.positions, wp_pose);
				wp_velo.clear();
			  control.get_cartesian_velocity(p.positions, p.velocities, wp_velo);
      }
      else
        stop = true;
    }
  }

LB_EXIT_MOVE:
	if(!b_jnt_state) {
		for(j=0; j<p.velocities.size(); j++)
		  p.velocities[j] = 0.0;
		control.move_velo(p.velocities);
	}
  if( fp1 )
    fclose(fp1);
  if( fp2 )
    fclose(fp2);
  printf("move_traj end\n");

	return !b_jnt_state;
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

