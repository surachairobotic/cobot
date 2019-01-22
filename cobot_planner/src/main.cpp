
#include "ros/ros.h"
#include "cobot_planner/cControl.h"
#include "cobot_planner/CobotPlanning.h"

cControl *p_control = NULL;
ros::Publisher pub_plan;

bool cobot_planning(cobot_planner::CobotPlanning::Request  &req
    , cobot_planner::CobotPlanning::Response &res);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_planner");
  cControl control("arm", "tool0");
  p_control = &control;
  
  try{
    control.init();
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("cobot_planning", cobot_planning);
    pub_plan = n.advertise<moveit_msgs::DisplayTrajectory>("/cobot/display_planned_path", 1, true);
    ROS_INFO("\nstart\n");
    
    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();
//    ros::spin();
  }
  catch(const std::string &err){
    ROS_ERROR("%s", err.c_str());
  }
  printf("end\n");
  ros::shutdown();
  return 0;
}


bool cobot_planning(cobot_planner::CobotPlanning::Request  &req
    , cobot_planner::CobotPlanning::Response &res){


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
}



