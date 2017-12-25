#ifndef __CCONTROL_H__
#define __CCONTROL_H__

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>



class cControl{

private:
  bool b_start_sub;
  std::string group_name, end_effector_name;
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner;
  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<double> joints;
  ros::Publisher pub_goal;
  ros::Subscriber sub_joints;
  sensor_msgs::JointState joint_state, goal;

  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup *joint_model_group;

private:
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void check_goal_state(const std::vector<double> &joints);
  robot_state::RobotState get_robot_state(const geometry_msgs::Pose &pose);

public:
  cControl(const std::string& _group_name, const std::string& _end_effector_name);
  void init();

  void clear_trajectory();
  bool plan_p2p(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);
  bool plan_p2p(const geometry_msgs::Pose &p2);
  bool plan_line(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, double step=0.002);
  bool plan_line(const geometry_msgs::Pose &p2, double step=0.002);
  bool plan_line(std::vector< geometry_msgs::Pose > &waypoints, double step=0.002);
  void replan_velocity(double velo, double acc);

  void move_pos_velo(const std::vector<double> &joint_pos, const std::vector<double> &joint_velo);
  void move_velo(const std::vector<double> &joint_velo);


  const geometry_msgs::Pose get_current_pose();
  const std::vector<double> get_current_joints();
  const geometry_msgs::Pose get_cartesian_position(const std::vector<double> &joint_pos);
  const std::vector<double> get_cartesian_velocity(const std::vector<double> &joint_pos
    , const std::vector<double> &joint_velo);
  inline const trajectory_msgs::JointTrajectory& get_trajectory(){ return trajectory.joint_trajectory; }
  inline bool is_valid_pose(const geometry_msgs::Pose &pose){
    try{
      get_robot_state(pose);
    }
    catch(int err){
      return false;
    }
    return true;
  }
  
  static void print_pose(const geometry_msgs::Pose &pose);
};

#endif
