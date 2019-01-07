#ifndef __CCONTROL_H__
#define __CCONTROL_H__

#include <mutex>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>



template <typename T> std::string tostr(const T& t) {
  std::ostringstream os;
  os<<t;
  return os.str();
}

#define mythrow(str) throw std::string(str) + "\n " +  __FILE__ + ":" + tostr(__LINE__)



class cControl{

private:
  bool b_start_sub;
  std::string group_name, end_effector_name, result_dir;
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner;
  moveit::planning_interface::MoveGroupInterface move_group;
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<double> joints;
  ros::Publisher pub_goal;
  ros::Subscriber sub_joints;
  ros::ServiceClient srv_set_acc, srv_set_p_gain, srv_set_i_gain, srv_set_goal_torque;
  sensor_msgs::JointState joint_state, goal;
  std::mutex mutex_joint_state;
  bool b_new_joint_state;  // check if the new one come

  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup *joint_model_group;

private:
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void check_goal_state(const std::vector<double> &joints);
  const robot_state::RobotStatePtr get_robot_state(const geometry_msgs::Pose &pose);

public:
  cControl();
  cControl(const std::string& _group_name, const std::string& _end_effector_name);
  void init();

  void clear_trajectory();
  bool plan_p2p(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);
  bool plan_p2p(const geometry_msgs::Pose &p2);
  bool plan_line(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, double step=0.002);
  bool plan_line(const geometry_msgs::Pose &p2, double step=0.002);
  bool plan_line(std::vector< geometry_msgs::Pose > &waypoints, double step=0.002);
  void replan_velocity(double velo, double acc);

	void move_pos_velo_acc(const std::vector<double> &joint_pos, const std::vector<double> &joint_velo, const std::vector<double> &joint_acc);
  void move_pos_velo(const std::vector<double> &joint_pos, const std::vector<double> &joint_velo);
  void move_velo(const std::vector<double> &joint_velo);
	void move_velo_acc(const std::vector<double> &joint_velo, const std::vector<double> &joint_acc);
  void set_acc(const std::vector<double> &acc);
  bool set_p_gain(std::string jnt, int _p_gain);
  int set_p_gain(std::string jnt);
  bool set_i_gain(std::string jnt, int _i_gain);
  int set_i_gain(std::string jnt);
	void set_goal_torque(std::string jnt, double torque);

  const geometry_msgs::Pose get_current_pose(); // get xyz position
  const std::vector<double> get_current_joints(); // get joint angles
  
  // get all joints'angle, velo and load
  bool get_last_joint_state(sensor_msgs::JointState *p_joint_state); // get the last state
  bool get_new_joint_state(sensor_msgs::JointState *p_joint_state); // return false if the new state does not come
  bool wait_new_joint_state(sensor_msgs::JointState *p_joint_state, double timeout); // wait until the new one come or timeout
  
  // FK
  void get_joint_positions(const geometry_msgs::Pose &pose, std::vector<double> &joint_pos);
  // IK
  void get_cartesian_position(const std::vector<double> &joint_pos, geometry_msgs::Pose &pose);
  void get_cartesian_velocity(const std::vector<double> &joint_pos
    , const std::vector<double> &joint_velo, std::vector<double> &xyz_velo);
  
//  const geometry_msgs::Pose get_cartesian_position(const std::vector<double> &joint_pos);
//  const std::vector<double> get_cartesian_velocity(const std::vector<double> &joint_pos
//    , const std::vector<double> &joint_velo);
  inline const trajectory_msgs::JointTrajectory& get_trajectory(){ return trajectory.joint_trajectory; }
  inline const moveit_msgs::RobotTrajectory& get_robot_trajectory(){ return trajectory; }
  bool is_valid_pose(const geometry_msgs::Pose &pose);
  
  void print_joints(robot_state::RobotStatePtr state);
  static void print_pose(const geometry_msgs::Pose &pose);
  
};

#endif
