

#include "cobot_sample_controller/cControl.h"
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>


cControl::cControl(const std::string& _group_name, const std::string& _end_effector_name)
    :spinner(1), group_name(_group_name), end_effector_name(_end_effector_name)
  ,move_group(group_name), b_start_sub(false), robot_model_loader("robot_description"){}


void cControl::init(){
  spinner.start();
  move_group.setEndEffectorLink(end_effector_name);
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  joint_model_group = kinematic_model->getJointModelGroup (group_name);
  
  
  ros::NodeHandle n;
  pub_goal = n.advertise<sensor_msgs::JointState>("test_dynamixel/goal", 1000);
/*  sub_joints = n.subscribe("joint_states", 10
    , &cControl::joint_states_callback, this);
  
  ros::Rate r(10);
  ROS_INFO("waiting for joint_states ...\n");
  while (ros::ok()){
    ros::spinOnce();
    if( b_start_sub ){
      break;
    }
    r.sleep();
  }
  if( !b_start_sub ){
    ROS_INFO("stopped by user\n");
    throw 0;
  }*/
  /*
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  */
}

void cControl::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg){
  b_start_sub = true;
  joint_state = *msg;
}


void cControl::clear_trajectory(){
  move_group.clearPathConstraints();
  trajectory.joint_trajectory.joint_names.clear();
  trajectory.joint_trajectory.points.clear();
}


bool cControl::plan_p2p(const geometry_msgs::Pose &p2){
  clear_trajectory();
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setPoseTarget(p2);
  if( !move_group.plan(plan) ){
    ROS_WARN("plan_p2p() : Could not compute plan successfully");
    return false;
  }
  trajectory = plan.trajectory_;
  return true;
}
/*
bool cControl::plan_p2p(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2){
  clear_trajectory();
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setPoseTarget(p2);
  if( !move_group.plan(plan) ){
    ROS_WARN("plan_p2p() : Could not compute plan successfully");
    return false;
  }
  trajectory = plan.trajectory_;
  return true;
}*/


bool cControl::plan_line(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, double step){
  move_group.setStartState(*move_group.getCurrentState());
  return plan_line(p2, step);
}

bool cControl::plan_line(const geometry_msgs::Pose &p2, double step){
  std::vector< geometry_msgs::Pose > wp;
  wp.push_back(p2);
  move_group.setStartState(*move_group.getCurrentState());
  return plan_line(wp, step);
}

bool cControl::plan_line(std::vector< geometry_msgs::Pose > &waypoints, double step){
  clear_trajectory();
  double fraction = move_group.computeCartesianPath(waypoints, step, 0.0, trajectory);
  if( fraction<1.0 ){
    ROS_WARN("plan_line() : Could not compute plan successfully");
    return false;
  }
  return true;
}



void cControl::check_goal_state(const std::vector<double> &joints){
  if( goal.name.empty() ){
    const trajectory_msgs::JointTrajectory &traj = get_trajectory();
    if( traj.joint_names.empty() ){
      ROS_ERROR("cControl::check_goal_state() : trajectory not found\n");
      throw 0;
    }
    goal.name = traj.joint_names;
  }
  if( goal.name.size()!=joints.size() ){
    ROS_ERROR("cControl::check_goal_state() : joint num does not match : %d / %d\n"
      , (int)joints.size(), (int)goal.name.size());
    throw 0;
  }
}



void cControl::move_pos_velo(const std::vector<double> &joint_pos, const std::vector<double> &joint_velo){
  check_goal_state(joint_pos);
  check_goal_state(joint_velo);
  goal.header.stamp = ros::Time::now();
  goal.position = joint_pos;
  goal.velocity = joint_velo;
  goal.effort.clear();
  pub_goal.publish(goal);
}

void cControl::move_velo(const std::vector<double> &joint_velo){
  check_goal_state(joint_velo);
  goal.header.stamp = ros::Time::now();
  goal.position.clear();
  goal.velocity = joint_velo;
  goal.effort.clear();
  pub_goal.publish(goal);
}



const std::vector<double> cControl::get_current_joints(){
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  if( !current_state ){
    ROS_ERROR("cControl::get_current_joints() : cannot get robot state\n");
    throw 0;
  }
  const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(group_name);
  current_state->copyJointGroupPositions(joint_model_group, joints);
  return joints;
}

const geometry_msgs::Pose cControl::get_current_pose(){
  return move_group.getCurrentPose().pose;
}


const geometry_msgs::Pose cControl::joints_to_pose(const std::vector<double> &joints){
  kinematic_state->setJointGroupPositions(joint_model_group, joints);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector_name);
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(end_effector_state, pose);
  return pose;
}



