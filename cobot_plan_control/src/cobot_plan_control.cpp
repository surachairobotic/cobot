#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "cobot_planner/cControl.h"
#include "cobot_planner/CobotPlanning.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include <numeric>
#include <actionlib/server/simple_action_server.h>
#include "cobot_msgs/ExecuteAction.h"
#include "cobot_msgs/ChangeMode.h"
#include "moveit_msgs/GetPositionIK.h"

#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
//using namespace std;
#include <cmath>

#include <cstddef>
#include "boost/date_time/posix_time/posix_time.hpp"

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

class ROSNode
{
public:
  ROSNode(ros::NodeHandle &_n) {
    n = &_n;
  };

  ros::NodeHandle *n;
};
ROSNode *rn;

void pose_callback(const geometry_msgs::PoseArray& msg);
void execute_callback(const std_msgs::Bool exe);
void stop_callback(const std_msgs::Bool msg);
bool func_plan_line(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, float step, double velo, double acc, moveit_msgs::RobotTrajectory &traj);
bool func_plan_line(const sensor_msgs::JointState &p1, const geometry_msgs::Pose& p2, float step, double velo, double acc, moveit_msgs::RobotTrajectory &traj);
bool func_plan_p2p(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, double velo, double acc, moveit_msgs::RobotTrajectory &traj);
bool move_trajectory_1(cControl &control);
bool move_trajectory_2(cControl &control);
bool move_trajectory_3(cControl &control);
double distance_point(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2);
double distance_quat(std::vector<double> &p1, std::vector<double> &p2);
//void print_move_cartesian(FILE *fp, double t_wp, const geometry_msgs::Point &wp, const std::vector<double> &wp_velo, double t_mv, const geometry_msgs::Point &mv, const std::vector<double> &mv_velo);
void print_move_cartesian(FILE *fp, double t_wp, const geometry_msgs::Point &wp, const std::vector<double> &wp_velo, double t_mv, const geometry_msgs::Point &mv, const std::vector<double> &mv_velo, const geometry_msgs::Point &desp, const std::vector<double> &des_velo, double t_lastPt, double err_dist);
void print_move_joint_1(FILE *fp, double t, const std::vector<double> &p_wp, const std::vector<double> &v_wp, const std::vector<double> &p_jnt, const std::vector<double> &v_jnt);
void print_move_joint_2(FILE *fp, const std::vector<double> &v_cal);
void print_trajectory(cControl &control, const std::string &file_name);
void print_info(int wp, const geometry_msgs::Pose &p, const geometry_msgs::Pose &cr);
void torque_callback(const std_msgs::Bool msg);
bool cobot_planning(cobot_planner::CobotPlanning::Request  &req, cobot_planner::CobotPlanning::Response &res);
sensor_msgs::JointState getIK(geometry_msgs::Pose &_ps, const sensor_msgs::JointState &_js);

void executeCB(const cobot_msgs::ExecuteGoalConstPtr &goal, actionlib::SimpleActionServer<cobot_msgs::ExecuteAction> *as);

std::string DRIVER_KEY = "";
cControl *p_control = NULL;
ros::Publisher pub_plan;
ros::Publisher pub_message;
std::string dir = "/home/mtec/catkin_ws/src/cobot/cobot_jig_controller/results/";
geometry_msgs::PoseArray p;
bool tq_over = false;
bool execute = false;
bool b_plan = false;
int sq = 0;
actionlib::SimpleActionServer<cobot_msgs::ExecuteAction>* as_ = NULL;
bool stop = false;

moveit_msgs::DisplayTrajectory dis_traj;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_plan_control");
  ros::NodeHandle n;
  rn = new ROSNode(n);

  ros::Rate loop_rate(10);

  try {
    cControl control("arm", "tool0");
    control.init();
    p_control = &control;

    ros::ServiceServer service = n.advertiseService("/cobot/planning", cobot_planning);
    ros::Subscriber sub_pose = n.subscribe("/cobot/pose_to_control", 1000, pose_callback);
    ros::Subscriber sub_execute = n.subscribe("/cobot/execute", 1000, execute_callback);
    ros::Subscriber sub_stop = n.subscribe("/cobot/control/stop", 10, stop_callback);
    pub_plan = n.advertise<moveit_msgs::DisplayTrajectory>("/cobot/display_planned_path", 1, true);
    pub_message = n.advertise<std_msgs::Bool>("/cobot/core_message", 10);
    ros::Subscriber sub_tq = n.subscribe("/cobot/torque_detection", 10, torque_callback);
    ros::ServiceClient srv_mode = n.serviceClient<cobot_msgs::ChangeMode>("/cobot/cobot_core/change_mode");
//    actionlib::SimpleActionServer<cobot_msgs::ExecuteAction> as_;
    actionlib::SimpleActionServer<cobot_msgs::ExecuteAction> action(n, "cobot_execute", boost::bind(&executeCB, _1, &action), false);
    as_ = &action;
    as_->start();

    sensor_msgs::JointState joint_state;
    geometry_msgs::Pose jnt_pose;
    cobot_msgs::ChangeMode msg;
    ROS_INFO("start...");
    while (ros::ok()) {
      if( execute ) {
      	msg.request.mode = "EXECUTE_MODE";
      	if (srv_mode.call(msg)) {
      		if(msg.response.error == "OK") {
      			DRIVER_KEY = msg.response.key;
            p_control->DRIVER_KEY = DRIVER_KEY;
      		}
      		else {
      			ROS_ERROR("msg.response.error is'n \"OK\"");
      			continue;
      		}
      	}
      	else {
      		ROS_ERROR("Failed to call service change_mode");
      		continue;
      	}

        execute = false;
        ROS_INFO("A");
        move_trajectory_1(*p_control);
        if(!tq_over) {
          std_msgs::Bool send;
          send.data = true;
          pub_message.publish(send);
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

void execute_callback(const std_msgs::Bool exe)
{
  ROS_INFO("execute_callback");
  if(exe.data and !execute)
    execute = true;
}

void stop_callback(const std_msgs::Bool msg) {
  stop = msg.data;
}

void pose_callback(const geometry_msgs::PoseArray& msg) {
  ROS_INFO("pose_callback");
  if( b_plan )
    return;
  p = msg;
  b_plan = true;
}

bool func_plan_line(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, float step, double velo, double acc, moveit_msgs::RobotTrajectory &traj) {
  bool result = false;
  result = p_control->plan_line(p1, p2, step);
  if(!result)
    return false;
  p_control->replan_velocity(velo, acc);
  traj = p_control->get_robot_trajectory();
  return result;
}
bool func_plan_line(const sensor_msgs::JointState &p1, const geometry_msgs::Pose& p2, float step, double velo, double acc, moveit_msgs::RobotTrajectory &traj) {
  bool result = false;
  result = p_control->plan_line(p1, p2, step);
  if(!result)
    return false;
  p_control->replan_velocity(velo, acc);
  traj = p_control->get_robot_trajectory();
  return result;
}

bool func_plan_p2p(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, double velo, double acc, moveit_msgs::RobotTrajectory &traj) {
  bool result = false;
  result = p_control->plan_p2p(p1, p2);
  if(!result)
    return false;
  p_control->replan_velocity(velo, acc);
  traj = p_control->get_robot_trajectory();
  return result;
}

bool cobot_planning(cobot_planner::CobotPlanning::Request  &req, cobot_planner::CobotPlanning::Response &res) {
  for(int i=0; i<req.pose_array.size(); i++) {
    if( !p_control->is_valid_pose(req.pose_array[i]) ){
      ROS_ERROR("Invalid pose_array[%d] : ", i);
      cControl::print_pose(req.pose_array[i]);
      return false;
    }
  }

  sensor_msgs::JointState joint_state;
  moveit_msgs::DisplayTrajectory dis_traj;
  dis_traj.trajectory.clear();
  moveit_msgs::RobotTrajectory robot_traj;
  bool ret;
  res.error_code = 0;
  for(int i=1 ; i<req.pose_array.size(); i++) {
    ROS_INFO("Plan [%d-%d]", i-1, i);
    // if( i == req.pose_array.size()-1 )
    if(i == 1)
      joint_state = req.seed;
    else {
      int sz1 = dis_traj.trajectory.size()-1;
      int sz2 = dis_traj.trajectory[sz1].joint_trajectory.points.size()-1;
      joint_state.name = {"J1", "J2", "J3", "J4", "J5", "J6"};
      joint_state.position = dis_traj.trajectory[sz1].joint_trajectory.points[sz2].positions;
      joint_state.velocity = dis_traj.trajectory[sz1].joint_trajectory.points[sz2].velocities;
      joint_state.effort = dis_traj.trajectory[sz1].joint_trajectory.points[sz2].effort;
    }
    sensor_msgs::JointState a = getIK(req.pose_array[i-1], joint_state);
    sensor_msgs::JointState b = getIK(req.pose_array[i], joint_state);
    ret = func_plan_line(a, req.pose_array[i], req.step_time, req.max_velocity, req.max_acceleration, robot_traj);

    ROS_INFO("P[%d-%d] A | %lf, %lf, %lf, %lf, %lf, %lf", i-1, i, a.position[0], a.position[1], a.position[2], a.position[3], a.position[4], a.position[5]);
    ROS_INFO("B | %lf, %lf, %lf, %lf, %lf, %lf", b.position[0], b.position[1], b.position[2], b.position[3], b.position[4], b.position[5]);
    // else if( req.type=="line" )
    //   ret = func_plan_p2p(req.pose_array[i-1], req.pose_array[i], req.max_velocity, req.max_acceleration, robot_traj);
    // else {
      // ret = func_plan_p2p(req.pose_array[i-1], req.pose_array[i], req.max_velocity, req.max_acceleration, robot_traj);
      // ROS_ERROR("Invalid plan type : %s", req.type.c_str());
      // return false;
    // }
    if(ret) {
      dis_traj.trajectory.push_back(robot_traj);
    }
    else {
      res.error_code = -1;
      return ret;
    }
  }

  for(int j=0; j<req.pose_array.size(); j++) {
    getIK(req.pose_array[j], joint_state);
  }

  for(int i=1; i<dis_traj.trajectory.size(); i++) {
    dis_traj.trajectory[i-1].joint_trajectory.points.pop_back();
    dis_traj.trajectory[i].joint_trajectory.points.erase(dis_traj.trajectory[i].joint_trajectory.points.begin());
  }

  ros::Duration ex(0, 0);
  for(int i=1; i<dis_traj.trajectory.size(); i++) {
    int l = dis_traj.trajectory[i-1].joint_trajectory.points.size();
    ex = dis_traj.trajectory[i-1].joint_trajectory.points[l-1].time_from_start;
    for(int j=0; j<dis_traj.trajectory[i].joint_trajectory.points.size(); j++) {
      dis_traj.trajectory[i].joint_trajectory.points[j].time_from_start += ex;
    }
  }

  trajectory_msgs::JointTrajectory traj;
  traj = dis_traj.trajectory[0].joint_trajectory;
  for(int i=1; i<dis_traj.trajectory.size(); i++) {
    for(int j=0; j<dis_traj.trajectory[i].joint_trajectory.points.size(); j++) {
      traj.points.push_back(dis_traj.trajectory[i].joint_trajectory.points[j]);
    }
  }

  for(int i=1;i<traj.points.size();i++){
    // ROS_INFO("[%d]", i);
    for(int k=0; k<traj.points[i].positions.size(); k++) {
      int index = 0;
      double v[] = {traj.points[i].positions[k], traj.points[i].positions[k]+(M_PI*2.0), traj.points[i].positions[k]-(M_PI*2.0)};
      double d[] = {fabs(traj.points[i-1].positions[k]-v[0]),
                    fabs(traj.points[i-1].positions[k]-v[1]),
                    fabs(traj.points[i-1].positions[k]-v[2])};

      for(int g=1; g<3; g++){
        if(d[index] > d[g]) {
          index = g;
        }
      }
      if(index != 0) {
        ROS_ERROR("[%d] : [%lf, %lf, (i-1)%lf]", k, traj.points[i].positions[k], v[index], traj.points[i-1].positions[k]);
      }
      // ROS_WARN(" [%d] : (i)%.3lf, %.3lf, (i-1)%.3lf]", k, traj.points[i].positions[k], v[index], traj.points[i-1].positions[k]);
      traj.points[i].positions[k] = v[index];
    }
  }
  // ROS_WARN("END cControl::replan_velocity(double velo, double acc)");

  p_control->set_trajectory(traj);
  std::string f_name;
  f_name = "/home/mtec/catkin_ws/src/cobot/cobot_plan_control/traj" + std::to_string(sq) + ".txt";
  sq++;
  print_trajectory(*p_control, f_name);
  pub_plan.publish(dis_traj);
/*
  if( !p_control->is_valid_pose(req.start_pose) ){
    ROS_ERROR("Invalid start_pos : ");
    cControl::print_pose(req.start_pose);
    return false;
  }
  if( !p_control->is_valid_pose(req.end_pose) ){
    ROS_ERROR("Invalid end_pos : ");
    cControl::print_pose(req.end_pose);
    return false;
  }

  ROS_INFO("start pose : ");
  cControl::print_pose(req.start_pose);
  ROS_INFO("end pose : ");
  cControl::print_pose(req.end_pose);

  bool ret;
  if( req.type=="line" ){
    ret = p_control->plan_line(req.start_pose, req.end_pose, req.step_time);
  }
  else if( req.type=="p2p" ){
    ret = p_control->plan_p2p(req.start_pose, req.end_pose);
  }
  else{
    ROS_ERROR("Invalid plan type : %s", req.type.c_str());
    return false;
  }
  if( ret ){
    moveit_msgs::DisplayTrajectory dis_traj;
    p_control->replan_velocity( req.max_velocity, req.max_acceleration);
    p_control->get_display_trajectory(dis_traj);
    pub_plan.publish(dis_traj);
    p_control->print_trajectory("traj.txt");
    printf("Trajectory has been created successfully.\n\n");
    res.error_code = 0;
  }
  else{
    ROS_ERROR("Failed to create trajectory.\n\n");
    res.error_code = -1;
  }
  return ret;
*/
  return true;
}

void executeCB(const cobot_msgs::ExecuteGoalConstPtr &goal, actionlib::SimpleActionServer<cobot_msgs::ExecuteAction> *as) {
  ROS_INFO("executeCB");
  execute = true;
  cobot_msgs::ExecuteFeedback feedback_;
  feedback_.percent_complete = 0.75;
  as->publishFeedback(feedback_);
  cobot_msgs::ExecuteResult result_;
  as->setSucceeded(result_);
}

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// move_trajectory //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
bool move_trajectory_1(cControl &control) {
  printf("move_trajectory\n");
  FILE *fp1 = fopen( (dir + "move_cartesian.txt").c_str() , "wt");
  FILE *fp2 = fopen( (dir + "move_joint.txt").c_str() , "wt");
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  int i=0;
  trajectory_msgs::JointTrajectoryPoint p = traj.points[i];

  // set acc to 3.14 rad/sec^2
  std::vector<double> acc(6);
  for(int j=0;j<acc.size();j++)
    acc[j] = M_PI*4.0;
  control.set_acc(acc);

  sensor_msgs::JointState joint_state;
  if( !control.wait_new_joint_state(&joint_state, 1.0) ) return false; // exit program if the new state does not come

  geometry_msgs::Pose jnt_pose, wp_pose;
	std::vector<double> jnt_vel, wp_vel;
  control.get_cartesian_position(p.positions, wp_pose);

  control.move_pos_velo(p.positions, p.velocities);
  ros::Time t = ros::Time::now();

//  dt = p.time_from_start.toSec() - d_time_move;

  ROS_INFO("WP : %d/%d", i, traj.points.size()-1);
  stop = false;
  while( !stop && ros::ok() && !tq_over) {
    if( !control.wait_new_joint_state(&joint_state, 1.0) ) {
			goto LB_EXIT_MOVE; // exit program if the new state does not come
		}

    // for(int k=0; k<joint_state.position.size(); k++) {
    //   double s  = fabs(joint_state.position[k]-p.positions[k]);
    //   double v0 = joint_state.velocity[k];
      // double tt  = p.time_from_start.toSec() - (ros::Time::now()-t).toSec();
    //
    //   // s = (v0+v1)/2*t;
    //   // v1 = (s*2/t)-v0;
    //   // p.velocities[k] = (s*2/tt)-v0;
    //
    //   // s = (ut + 0.5g(t^2))
    //   // u = ( 0.5*a*(t^2) + s ) / t
    //   p.velocities[k] = ( 0.5 * acc[k] * (tt * tt) + s ) / tt;
    // }

    ROS_INFO("TIME : %lf", p.time_from_start.toSec() - (ros::Time::now()-t).toSec());

    // p.velocities[5] = M_PI/3.0;
    const double TH_DQ[] = {M_PI/6.0, M_PI/8.0, M_PI/8.0, M_PI/8.0, M_PI/4.0, M_PI/4.0};
    // if( i == traj.points.size()-1 )
    for(int k=0; k<p.velocities.size(); k++)
//      if(p.velocities[k] == 0.0)
//      p.velocities[k] = M_PI/8.0;
      if( fabs(p.velocities[k]) < TH_DQ[k] ){
        p.velocities[k] = p.velocities[k] >= 0.0 ? TH_DQ[k] : -TH_DQ[k];
      }
    control.move_pos_velo(p.positions, p.velocities);

    control.get_cartesian_position(joint_state.position, jnt_pose);
    if( i == traj.points.size()-1 ) {
      int count = 0;
      for(int k=0; k<joint_state.velocity.size(); k++) {
        if( joint_state.velocity[k] < 0.005 && distance_point(jnt_pose, wp_pose) < 0.005 && distance_quat(joint_state.position, p.positions) < 0.1 )
          count++;
      }
      if( count == 6 ) {
        stop = true;
      }
    }
    else if( distance_point(jnt_pose, wp_pose) < 0.05 /*&& distance_quat(joint_state.position, p.positions) < 0.1*/ ) {
      if( i < traj.points.size()-1 ) {
        i++;
        ROS_INFO("WP : %d/%d", i, traj.points.size()-1);
        p = traj.points[i];
        control.get_cartesian_position(p.positions, wp_pose);
      }
    }
  }

LB_EXIT_MOVE:
  printf("move_traj end\n");

	return true;
}

bool move_trajectory_2(cControl &control) {
  printf("move_trajectory_2\n");
  FILE *fp1 = fopen( (dir + "move_cartesian.txt").c_str() , "wt");
  FILE *fp2 = fopen( (dir + "move_joint.txt").c_str() , "wt");
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();

  char target_str[512];

  ros::Rate r(100);
  ros::Time t_start = ros::Time::now(), t_print = t_start, start_time, read_time, run_time;

  // set acc to 3.14 rad/sec^2
  std::vector<double> acc(6);
  for(int j=0;j<acc.size();j++)
    acc[j] = 2.0*M_PI;
  control.set_acc(acc);

	bool first_time = true;
  sensor_msgs::JointState joint_state, j_state;
  int i = 1;
  trajectory_msgs::JointTrajectoryPoint p = traj.points[i];
  geometry_msgs::Pose wp_pose, current_pose, ex_pose, pose, tmp_pose;
	std::vector<double> wp_velo, mv_velo, wp_j_velo;
  ros::Time t = ros::Time::now();

	wp_j_velo.clear();
	for(int j=0; j<p.velocities.size(); j++)
		wp_j_velo.push_back(p.velocities[j]);

  if( !control.wait_new_joint_state(&joint_state, 1.0) ) return false; // exit program if the new state does not come
  control.get_cartesian_position(joint_state.position, current_pose);
  control.get_cartesian_position(p.positions, wp_pose);
  control.get_cartesian_velocity(p.positions, p.velocities, wp_velo);
  control.get_cartesian_velocity(joint_state.position, joint_state.velocity, mv_velo);
//  std::vector<double> current_joint = control.get_current_joints();

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Loop  ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
  stop = false;
  bool b_jnt_state = false;
	double dis_min, dt, limit_vel = M_PI / 4.0, sum, d_lab, d_time_move;
	int j, k;
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
		//print_move_cartesian(fp1, p.time_from_start.toSec(), wp_pose.position, wp_velo, d_time_move, current_pose.position, mv_velo);

    double dist_pnt = distance_point(current_pose, wp_pose);
    double dist_jnt = 0.00;
    for( int l=0; l<joint_state.position.size(); l++) {
      dist_jnt += fabs(joint_state.position[l]-p.positions[l]);
    }
    if( dist_pnt < 0.005 && dist_jnt < 0.05) {
      if( i < traj.points.size()-1 ) {
        i++;
        p = traj.points[i];
				wp_j_velo.clear();
				for(int j=0; j<p.velocities.size(); j++)
					wp_j_velo.push_back(p.velocities[i]);
        control.get_cartesian_position(p.positions, wp_pose);
				wp_velo.clear();
			  control.get_cartesian_velocity(p.positions, p.velocities, wp_velo);
      }
      else
        stop = true;
    }
    else if( dist_pnt > 0.075 ) {
			b_jnt_state = false;
  		goto LB_EXIT_MOVE; // exit program if the new state does not come
    }

		dis_min = 999.999;
		dt = p.time_from_start.toSec() - d_time_move;
		//ROS_INFO("dt, p.time, read-start | %lf, %lf, %lf", dt, p.time_from_start.toSec(), (read_time - start_time).toSec());
		if(dt < 0.02) {
			ROS_WARN("dt : %lf", dt);
			dt = 0.02;
		}
    else
			ROS_INFO("dt : %lf", dt);

		print_move_joint_1(fp2, d_time_move, p.positions, wp_j_velo, joint_state.position, joint_state.velocity);

		double h_dt = fabs(dt/2.0);
		for(int j=0; j<p.velocities.size(); j++) {
			double s = fabs(fabs(p.positions[j])-fabs(joint_state.position[j]));
			double u = fabs(joint_state.velocity[j]);
//			ROS_INFO("vel[%d]:%lf, s:%lf, u:%lf, dt:%lf", j, ((4*s)/(3*dt)) + u, s, u, dt);
//			ROS_INFO("vel[%d]:%lf", j, p.velocities[j]);
//			p.velocities[j] = ((4*s)/(3*dt)) + u;
//			p.velocities[j] = s+(u/2) / (3*dt/4);
//			p.velocities[j] = (s-(u*h_dt)-(0.5*M_PI*h_dt*h_dt)) / h_dt;
			p.velocities[j] = ( p.positions[j] - joint_state.position[j] ) / dt;
			ROS_INFO("vel[%d]:%lf", j, p.velocities[j]);
			if( p.velocities[j] > limit_vel )	p.velocities[j] = limit_vel;
			else if( p.velocities[j] < -limit_vel )	p.velocities[j] = -limit_vel;
		}

		ROS_INFO("A");
		for(int j=0; j<p.velocities.size(); j++) {
      /*
			if( p.positions[j] > joint_state.position[j] )
				p.velocities[j] = p.velocities[j] >= 0.00 ? p.velocities[j] : -p.velocities[j];
			else
				p.velocities[j] = p.velocities[j] >= 0.00 ? -p.velocities[j] : p.velocities[j];
      */
      if(p.velocities[j] < 0.00)
        p.velocities[j] = -p.velocities[j];
		}

		ROS_INFO("B");
		print_move_joint_2(fp2, p.velocities);

//		for(int i=0;i<p.accelerations.size();i++)
//		  acc[i] = p.accelerations[i];
//		control.set_acc(acc);

		ROS_INFO("C");
//		for(int i=0;i<p.velocities.size();i++)
//		  p.velocities[i] = 0.000;
//    tq_over = false; // for debug
    if(tq_over) {
			b_jnt_state = false;
  		goto LB_EXIT_MOVE; // exit program if the new state does not come
    }
    control.move_pos_velo(p.positions, p.velocities);
    // control.move_velo(p.velocities);
//    control.move_velo_acc(p.velocities, acc);
		ROS_INFO("D");
		sum = 0.0;
		do {
		  if( !control.wait_new_joint_state(&j_state, 1.0) ) {
				b_jnt_state = true;
				goto LB_EXIT_MOVE; // exit program if the new state does not come
			}
			for(int k=0; k<joint_state.position.size(); k++)
				sum += fabs( joint_state.position[k] - j_state.position[k] );
		} while(sum < 0.0001);
		ROS_INFO("E");
		if(first_time) {
			start_time = ros::Time::now();
			first_time = false;
		}
		ROS_INFO("F");
  }

LB_EXIT_MOVE:
	if(!b_jnt_state) {
		for(int j=0; j<p.velocities.size(); j++)
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
double distance_quat(std::vector<double> &p1, std::vector<double> &p2) {
  double sum = 0.0;
  for(int i=0; i<6; i++) {
    sum += fabs(p1[i]-p2[i]);
  }
  return sum;
}

/*
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
*/

void print_move_cartesian(FILE *fp, double t_wp, const geometry_msgs::Point &wp, const std::vector<double> &wp_velo, double t_mv, const geometry_msgs::Point &mv, const std::vector<double> &mv_velo, const geometry_msgs::Point &desp, const std::vector<double> &des_velo, double t_lastPt, double err_dist)
{
	fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf"
						, t_wp, t_mv, wp.x, wp.y, wp.z, mv.x, mv.y, mv.z);
	for(int i=0; i<3; i++)
		fprintf(fp, " %lf", wp_velo[i]);
	for(int i=0; i<3; i++)
		fprintf(fp, " %lf", mv_velo[i]);
	fprintf(fp, " %lf %lf %lf"
						, desp.x, desp.y, desp.z);
	for(int i=0; i<3; i++)
		fprintf(fp, " %lf", des_velo[i]);

	fprintf(fp, " %lf %lf", t_lastPt, err_dist);
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

void print_trajectory(cControl &control, const std::string &file_name) {
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  FILE *fp = fopen(file_name.c_str(), "wt");
  if( !fp )
    ROS_ERROR("Cannot create file result file : %s", file_name.c_str());

  // for(int i=0; i<traj.joint_names.size(); i++)
  //   printf("joint : %s\n", traj.joint_names[i].c_str());

  for(int i=0; i<traj.points.size(); i++) {
    const trajectory_msgs::JointTrajectoryPoint &p = traj.points[i];
    geometry_msgs::Pose pose;
    std::vector<double> velo;
    control.get_cartesian_position(p.positions, pose);
    control.get_cartesian_velocity(p.positions, p.velocities, velo );
    // printf("point [%d]  time : %lf\n", i, p.time_from_start.toSec() );
    // printf("angle :");
    if( fp )
        fprintf(fp, "%lf", p.time_from_start.toSec());
    for(int j=0; j<p.positions.size(); j++) {
      // printf(" %.3lf", p.positions[j]);
      if( fp )
        fprintf(fp, " %lf", p.positions[j]);
    }
    // printf("\nangular velo :");
    for(int j=0; j<p.velocities.size(); j++) {
      // printf(" %.3lf", p.velocities[j]);
      if( fp )
        fprintf(fp, " %lf", p.velocities[j]);
    }
    // printf("\nangular acc :");
    for(int j=0; j<p.accelerations.size(); j++) {
      // printf(" %.3lf", p.accelerations[j]);
      if( fp )
        fprintf(fp, " %lf", p.accelerations[j]);
    }
/*    printf("\neff :");
    for(int j=0;j<p.effort.size();j++){
      printf(" %.3lf", p.effort[j]);
      if( fp )
        fprintf(fp, " %lf", p.effort[j]);
    }*/

    // printf("\nxyz : %.3lf %.3lf %.3lf\n", pose.position.x, pose.position.y, pose.position.z);
    // printf("quart : %.3lf %.3lf %.3lf %.3lf\n", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    // printf("velo : ");
    // for(int i=0; i<velo.size(); i++)
    //   printf(" %.3lf", velo[i]);
    // printf("\n");
    if( fp ){
      fprintf(fp, " %lf %lf %lf %lf %lf %lf %lf"
        , pose.position.x, pose.position.y, pose.position.z
        , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      for(int i=0; i<velo.size(); i++)
        fprintf(fp, " %lf", velo[i]);
      fprintf(fp, "\n");
    }
  }
  if(fp)
    fclose(fp);

  // printf("\nstart - end point\n");
  // const int ii[] = { 0, (int)traj.points.size()-1 };
  // for(int i=0; i<2; i++){
  //   const trajectory_msgs::JointTrajectoryPoint &p = traj.points[ii[i]];
  //   geometry_msgs::Pose pose;
  //   control.get_cartesian_position(p.positions, pose);
  //   printf("xyz : %.3lf %.3lf %.3lf, w : %.3lf, %.3lf, %.3lf, %.3lf\n"
  //     , pose.position.x, pose.position.y, pose.position.z
  //     , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  // }
}

void print_info(int wp, const geometry_msgs::Pose &p, const geometry_msgs::Pose &cr) {
//	printf("WP[%d]", wp);
	printf("[%d]PP - XYZ[%lf, %lf, %lf]", wp,  p.position.x,  p.position.y,  p.position.z);
	printf("\n");
//	printf("[%d]EX - XYZ[%lf, %lf, %lf]", wp, ex.position.x, ex.position.y, ex.position.z);
//	printf("\n");
	printf("[%d]CR - XYZ[%lf, %lf, %lf]", wp, cr.position.x, cr.position.y, cr.position.z);
	printf("\n");
}

void torque_callback(const std_msgs::Bool msg) {
  tq_over = msg.data;
  return;
}


/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// move_trajectory //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
bool move_trajectory_3(cControl &control) {
  printf("move_trajectory\n");
  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

  FILE *fp1 = fopen( (dir + iso_time_str + "move_cartesian.txt").c_str() , "wt");
  FILE *fp2 = fopen( (dir + iso_time_str + "move_joint.txt").c_str() , "wt");
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();

  char target_str[512];

  ros::Rate r(100);
  ros::Time t_start = ros::Time::now(), t_print = t_start, start_time, read_time, run_time;

  // set acc to 3.14 rad/sec^2
  std::vector<double> acc(6), vel_command(6);

  double err_distance;
  for(int j=0;j<acc.size();j++) {
    acc[j] = M_PI;
    vel_command[j] = 0.0;
  }
  control.set_acc(acc);

	bool first_time = true;
  sensor_msgs::JointState joint_state, j_state;
  int i = 1;
  trajectory_msgs::JointTrajectoryPoint p = traj.points[i];
  trajectory_msgs::JointTrajectoryPoint last_p = traj.points[i-1];
  trajectory_msgs::JointTrajectoryPoint interp_p= traj.points[i];
  geometry_msgs::Pose wp_pose, current_pose, ex_pose, pose, tmp_pose, des_pose;
  std::vector<double> wp_velo, mv_velo, wp_j_velo, des_velo;
  ros::Time t = ros::Time::now();

    ROS_INFO("vel_wp : %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",p.velocities[0], p.velocities[1], p.velocities[2], p.velocities[3], p.velocities[4], p.velocities[5]);

	wp_j_velo.clear();
	for(int j=0; j<p.velocities.size(); j++)
		wp_j_velo.push_back(p.velocities[j]);

  if( !control.wait_new_joint_state(&joint_state, 1.0) ) return false; // exit program if the new state does not come
//  control.get_cartesian_position(joint_state.position, current_pose);
//  control.get_cartesian_position(p.positions, wp_pose);
//  control.get_cartesian_velocity(p.positions, p.velocities, wp_velo);
//  control.get_cartesian_velocity(joint_state.position, joint_state.velocity, mv_velo);
//  std::vector<double> current_joint = control.get_current_joints();

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Loop  ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
  stop = false;
  bool b_jnt_state = false;
	double dis_min, dt, limit_vel = M_PI / 2.0, sum, d_lab, d_time_move, kp=0.3, kpp=0.7, ki=0.01, kd=0.0;
	double last_t_nextPt = 0.0;
	int j, k, count = 0;
	double treshold = 0.0;

	std::vector<std::vector<double>> v_sum;
  std::vector<double> v_tmp(1);
  v_tmp[0] = 0.0;
  std::vector<std::vector<double>> sum_err;
	std::vector<double> old_err(6);

	for(int j=0; j<old_err.size(); j++) {
		old_err[j] = 0.0;
		v_sum.push_back(v_tmp);
  }

  while( !stop && ros::ok() ) {
    if( !control.wait_new_joint_state(&joint_state, 1.0) ) {
			b_jnt_state = true;
			goto LB_EXIT_MOVE; // exit program if the new state does not come
		}

		run_time = read_time;
		read_time = ros::Time::now();
		if(first_time) start_time = read_time;
		d_lab = (read_time-run_time).toSec()*1000;
		d_time_move = (read_time-start_time).toSec(); // time already move from start
		if(d_lab >= 42.5)
			ROS_WARN("[%.2lf s.][%.2lf s.] : %.2lf ms.", p.time_from_start.toSec(), d_time_move, d_lab);
		else
			ROS_INFO("[%.2lf s.][%.2lf s.] : %.2lf ms.", p.time_from_start.toSec(), d_time_move, d_lab);

		for(int j=0; j<p.positions.size(); j++) {
			interp_p.positions[j] = last_p.positions[j]+((p.positions[j]-last_p.positions[j])*(d_time_move-last_t_nextPt)/(p.time_from_start.toSec()-last_p.time_from_start.toSec()));
			interp_p.velocities[j] = last_p.velocities[j]+((p.velocities[j]-last_p.velocities[j])*(d_time_move-last_t_nextPt)/(p.time_from_start.toSec()-last_p.time_from_start.toSec()));
		} // interpolate positions and velocities at current time

		dis_min = 999.999;
		dt = p.time_from_start.toSec() - d_time_move; // waypoint time - time already move from start
		//ROS_INFO("dt, p.time, read-start | %lf, %lf, %lf", dt, p.time_from_start.toSec(), (read_time - start_time).toSec());
		if(dt < 0.02) {
			ROS_WARN("dt : %lf", dt);
			dt = 0.02;
		}

		for(int j=0; j<p.accelerations.size(); j++)
		  acc[j] = ( p.velocities[j]-joint_state.velocity[j] ) / dt;
//		control.set_acc(acc);

    for(int j=0; j<p.velocities.size(); j++) {
		  //double target = p.velocities[j];
			//double err_pos = (p.positions[j] - joint_state.position[j]);

			double target = interp_p.velocities[j];
			double err_pos = (interp_p.positions[j] - joint_state.position[j]);
			double err = target - joint_state.velocity[j];
			v_sum[j].push_back( ((old_err[j]+err)/2)*dt );

		  if(v_sum[j].size() > 10)
			  v_sum[j].erase(v_sum[j].begin());
		  sum = 0.0;
		  for(int l=0; l<v_sum[j].size(); l++)
			  sum += v_sum[j][l];

		  //vel_command[j] = target + (kp*err) + (ki*sum) + (kd*(old_err[j]-err)/dt);
		  vel_command[j] = target + (kp*err) + (kpp*err_pos)+ (ki*sum);
		  old_err[j] = err;
		}

//    ROS_INFO("vel_com : %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",vel_command[0], vel_command[1], vel_command[2], vel_command[3], vel_command[4], vel_command[5]);

/*		for(int i=0; i<vel_command.size(); i++) {
			if( p.positions[i] > joint_state.position[i] )
				vel_command[i] = vel_command[i] >= 0.0 ? vel_command[i] : -vel_command[i];
			else
				vel_command[i] = vel_command[i] >= 0.0 ? -vel_command[i] : vel_command[i];
		}
*/

    // control.move_velo_acc(vel_command, acc);

    if(traj.points.size()-1 == i){
      for(int j=0; j<p.velocities.size(); j++) {
        if (interp_p.velocities[j] == 0.0)
          vel_command[j] = 0.0005;
      }
      control.move_pos_velo(p.positions, vel_command);
      ROS_INFO("vel_com : %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf",vel_command[0], vel_command[1], vel_command[2], vel_command[3], vel_command[4], vel_command[5]);
    }
    else{
      control.move_velo_acc(vel_command, acc);
      ROS_WARN("acc : %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf", acc[0], acc[1], acc[2], acc[3], acc[4], acc[5]);
//      control.move_velo(vel_command);
    }

		control.get_cartesian_position(joint_state.position, current_pose);
		control.get_cartesian_velocity(joint_state.position, joint_state.velocity, mv_velo);
		control.get_cartesian_position(p.positions, wp_pose);
		control.get_cartesian_position(interp_p.positions, des_pose);
		control.get_cartesian_velocity(p.positions, p.velocities, wp_velo);
		control.get_cartesian_velocity(interp_p.positions, interp_p.velocities, des_velo);
    err_distance = distance_point(current_pose, wp_pose);

		//print_move_cartesian(fp1, p.time_from_start.toSec(), wp_pose.position, wp_velo, d_time_move, current_pose.position, mv_velo);
		print_move_cartesian(fp1, p.time_from_start.toSec(), wp_pose.position, wp_velo, d_time_move, current_pose.position, mv_velo, des_pose.position, des_velo, last_t_nextPt, err_distance);

		print_move_joint_1(fp2, d_time_move, p.positions, p.velocities, joint_state.position, joint_state.velocity);
		print_move_joint_2(fp2, vel_command);

    ROS_WARN("WP : %d, %d", i, traj.points.size()-1);

		if(first_time) {
			start_time = ros::Time::now();
			first_time = false;
		}

    /*if( i == traj.points.size()-1 ) {
       control.move_pos_velo(p.positions, vel_command);
    }*/


		ROS_INFO("xyz[%.2lf %.2lf %.2lf]:[%.2lf %.2lf %.2lf]", wp_pose.position.x, wp_pose.position.y, wp_pose.position.z, current_pose.position.x, current_pose.position.y, current_pose.position.z);
		ROS_INFO("dist : %lf", distance_point(current_pose, wp_pose));
		if (i == 1)
		  treshold = 0.008;
		else
		  treshold = 0.005;

    double dist_pnt = distance_point(current_pose, wp_pose);
    if( dist_pnt < treshold ) {
      if( i < traj.points.size()-1 ) {
        i++;
        p = traj.points[i];
        last_p = traj.points[i-1];
        last_t_nextPt = d_time_move;
				wp_j_velo.clear();
				for(int j=0; j<p.velocities.size(); j++)
					wp_j_velo.push_back(p.velocities[j]);
        control.get_cartesian_position(p.positions, wp_pose);
				wp_velo.clear();
			  control.get_cartesian_velocity(p.positions, p.velocities, wp_velo);
      }
      else{
        stop = true;
        ROS_INFO("stop = true");
//        for(j=0; j<p.velocities.size(); j++)
//		      p.velocities[j] = 0.0;
//		      control.move_velo(p.velocities);
		  }
    }
    else if( dist_pnt > 0.075 ) {
			b_jnt_state = false;
  		goto LB_EXIT_MOVE; // exit program if the new state does not come
    }
  }

LB_EXIT_MOVE:
	if(!b_jnt_state) {
		for(int j=0; j<p.velocities.size(); j++)
		  p.velocities[j] = 0.0;
		control.move_velo(p.velocities);
	}
  printf("set zero end\n");
  if( fp1 )
    fclose(fp1);
  if( fp2 )
    fclose(fp2);
  printf("move_traj end\n");

	return !b_jnt_state;
}

sensor_msgs::JointState getIK(geometry_msgs::Pose &_ps, const sensor_msgs::JointState &_js) {
	moveit_msgs::GetPositionIK msg;
  msg.request.ik_request.group_name = "arm";
  msg.request.ik_request.robot_state.joint_state = _js;
  msg.request.ik_request.pose_stamped.pose = _ps;
  // msg.request.ik_request.timeout = 1;
  msg.request.ik_request.attempts = 0;

  ros::ServiceClient srv_ik = rn->n->serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
	if(srv_ik.call(msg))
	{
		; //msg.response.solution.joint_state;
	}
	else {
		ROS_ERROR("Failed to call service compute_fk");
	}

  ROS_INFO("JS : %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", msg.response.solution.joint_state.position[0]
                                                    , msg.response.solution.joint_state.position[1]
                                                    , msg.response.solution.joint_state.position[2]
                                                    , msg.response.solution.joint_state.position[3]
                                                    , msg.response.solution.joint_state.position[4]
                                                    , msg.response.solution.joint_state.position[5]);
  return msg.response.solution.joint_state;
}
