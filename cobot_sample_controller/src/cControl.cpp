

#include "cobot_sample_controller/cControl.h"
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>


#define POW2(x) ((x)*(x))

cControl::cControl(const std::string& _group_name, const std::string& _end_effector_name)
    :spinner(1), group_name(_group_name), end_effector_name(_end_effector_name)
  ,move_group(group_name), b_start_sub(false), robot_model_loader("robot_description")
  ,joint_model_group(NULL)
{
}


void cControl::init(){
  spinner.start();
  move_group.setEndEffectorLink(end_effector_name);
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  joint_model_group = kinematic_model->getJointModelGroup (group_name);


  ros::NodeHandle n;
  pub_goal = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/goal", 1000);

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

bool cControl::plan_p2p(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2){
  clear_trajectory();
  robot_state::RobotState rb = get_robot_state(p1);
/*  {
    const Eigen::Affine3d &end_effector_state = rb.getGlobalLinkTransform(end_effector_name);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(end_effector_state, pose);
    printf("start rb xyz : %.3lf %.3lf %.3lf, w : %.3lf, %.3lf, %.3lf, %.3lf\n"
      , pose.position.x, pose.position.y, pose.position.z
      , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }*/
  move_group.setStartState(rb);
  move_group.setPoseTarget(p2);
  {
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "link_6";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1;
/*    ocm.orientation.w = p1.orientation.w;
    ocm.orientation.x = p1.orientation.x;
    ocm.orientation.y = p1.orientation.y;
    ocm.orientation.z = p1.orientation.z;*/
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints con;
    con.orientation_constraints.push_back(ocm);
//    move_group.setPathConstraints(con);
    move_group.setPlanningTime(10.0);
  }

  if( !move_group.plan(plan) ){
    ROS_WARN("plan_p2p() : Could not compute plan successfully");
    return false;
  }
  trajectory = plan.trajectory_;
  return true;
}


bool cControl::plan_line(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, double step){
  std::vector< geometry_msgs::Pose > wp;
  wp.push_back(p2);
  move_group.setStartState(get_robot_state(p1));
  return plan_line(wp, step);

/*  //move_group.setStartState(*move_group.getCurrentState());
  printf("%lf, %lf, %lf\n", p1.position.x, p1.position.y, p1.position.z);
  move_group.setStartState(get_robot_state(p1));
  return plan_line(p2, step);*/
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
      mythrow("cControl::check_goal_state() : trajectory not found\n");
    }
    goal.name = traj.joint_names;
  }
  if( goal.name.size()!=joints.size() ){
    mythrow(std::string("cControl::check_goal_state() : joint num does not match : ") + 
      tostr((int)joints.size()) + " / " + tostr((int)goal.name.size()));
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
    mythrow("cControl::get_current_joints() : cannot get robot state\n");
  }
  const robot_state::JointModelGroup *joint_model_group = current_state->getJointModelGroup(group_name);
  current_state->copyJointGroupPositions(joint_model_group, joints);
  return joints;
}

const geometry_msgs::Pose cControl::get_current_pose(){
  return move_group.getCurrentPose().pose;
}


const geometry_msgs::Pose cControl::get_cartesian_position(const std::vector<double> &joint_pos){
  robot_state::RobotState robot_state(*move_group.getCurrentState());
  robot_state.setJointGroupPositions(joint_model_group, joint_pos);
  const Eigen::Affine3d &e = robot_state.getGlobalLinkTransform(end_effector_name);
  /*
  kinematic_state->setJointGroupPositions(joint_model_group, joint_pos);
  const Eigen::Affine3d &e = kinematic_state->getGlobalLinkTransform(end_effector_name);
  */
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(e, pose);
  if( pose.position.x<-10000 || pose.position.x>10000 ){
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
        printf("%lf ", e.matrix()(i,j));
      }
      printf("\n");
    }
    print_pose(pose);
    mythrow("cControl::get_cartesian_position() : invalid xyz\n");
  }
  return pose;
}

const std::vector<double> cControl::get_cartesian_velocity(const std::vector<double> &joint_pos
  , const std::vector<double> &joint_velo){
  kinematic_state->setJointGroupPositions(joint_model_group, joint_pos);
  Eigen::MatrixXd jacobian;
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(end_effector_name),
                             reference_point_position,
                             jacobian);
  Eigen::VectorXd dq(joint_velo.size()), dx;
  for(int i=dq.size()-1;i>=0;i--)
    dq(i) = joint_velo[i];
  dx = jacobian * dq;
  std::vector<double> dx2(dx.size());
  for(int i=dx2.size()-1;i>=0;i--)
    dx2[i] = dx(i);
  return dx2;
}

void cControl::replan_velocity(double velo, double acc){
  if( acc<=0.0 || velo<=0.0){
    ROS_ERROR("Invalid acc or velo : %.3lf, %.3lf\n", acc, velo);
    return;
  }
  struct tCartesianPose{
    geometry_msgs::Pose pose;
    geometry_msgs::Twist velocity;
  };
  double dis_all = 0.0;
  std::vector<tCartesianPose> traj_car;
  trajectory_msgs::JointTrajectory &traj = trajectory.joint_trajectory;
  traj_car.resize(traj.points.size());
  for(int i=0;i<traj.points.size();i++){
    const trajectory_msgs::JointTrajectoryPoint &p = traj.points[i];
    tCartesianPose &tc = traj_car[i];
    tc.pose = get_cartesian_position(p.positions);
    std::vector<double> velo = get_cartesian_velocity(p.positions, p.velocities );
    tc.velocity.linear.x = velo[0];
    tc.velocity.linear.y = velo[1];
    tc.velocity.linear.z = velo[2];
    tc.velocity.angular.x = velo[3];
    tc.velocity.angular.y = velo[4];
    tc.velocity.angular.z = velo[5];

    if( i > 0 )
      dis_all+= sqrt(
        POW2(tc.pose.position.x - traj_car[i-1].pose.position.x) +
        POW2(tc.pose.position.y - traj_car[i-1].pose.position.y) +
        POW2(tc.pose.position.z - traj_car[i-1].pose.position.z));
  }
  if( dis_all==0.0 ){
    ROS_ERROR("Trajectory distance is 0\n");
    return;
  }
  double t_acc = velo / acc,              // time used to accelerate to target velocity
    dis_acc = 0.5 * acc * POW2(t_acc),    // distance used to accelerate to target velocity
    t_all;
  double dis_end_acc, dis_start_dec, t_start_dec;
  if( dis_all < dis_acc * 2 ){
    ROS_WARN("Velocity will not reach the target velocity because the acceleration is too small\n");
    dis_end_acc = dis_start_dec = dis_all * 0.5;
    t_start_dec = sqrt(2 * dis_end_acc / acc);
    t_all = t_start_dec * 2;
  }
  else{
    dis_end_acc = dis_acc;
    dis_start_dec = dis_all - dis_acc;
    t_start_dec = t_acc + (dis_all - 2*dis_acc) / velo;
    t_all = t_start_dec + t_acc;
  }

  double dis = 0.0;
  for(int i=1;i<traj.points.size();i++){
    double v, t;
    tCartesianPose &tc = traj_car[i], &tc1 = traj_car[i-1];
    dis+= sqrt(POW2(tc.pose.position.x - tc1.pose.position.x) +
      POW2(tc.pose.position.y - tc1.pose.position.y) +
      POW2(tc.pose.position.z - tc1.pose.position.z));
    if( dis < dis_end_acc ){
      t = sqrt( 2 * dis / acc );   // s = 0.5*a*t^2
      v = acc * t;
    }
    else if( dis > dis_start_dec){
      t = sqrt( 2 * (dis_all - dis) / acc );
      v = acc * t;
      t = t_all - t;
    }
    else{
      v = velo;
      t = velo / acc + (dis - dis_acc) / velo;
    }
    double vj = sqrt( POW2(tc.velocity.linear.x)+POW2(tc.velocity.linear.y)+POW2(tc.velocity.linear.z) ),
      ratio = fabs(vj) < 0.0001 ? 0 : v / vj;
    for(int j=traj.points[i].velocities.size()-1;j>=0;j--){
      traj.points[i].velocities[j]*= ratio;
    }
    traj.points[i].time_from_start.fromSec(t);
  }
}

robot_state::RobotState cControl::get_robot_state(const geometry_msgs::Pose &pose){
/*  Eigen::Affine3d e2 = kinematic_state->getGlobalLinkTransform(end_effector_name), end_effector_state;
  end_effector_state = e2;
  end_effector_state.translation() << end_effector_state.translation().x() + 0.15
    , end_effector_state.translation().y(), end_effector_state.translation().z();
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
//  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
//  ROS_INFO("found : %d", (int)found_ik);
  */
  /*
  robot_state::RobotState robot_state(*move_group.getCurrentState());
//  robot_state.setToDefaultValues();
  if( !robot_state.setFromIK( joint_model_group
    , pose, end_effector_name, 10, 0.1) ){
    ROS_ERROR("cControl::get_robot_state : setFromIK failed\n");
    throw 0;
  }*/
  /*
  robot_state::RobotState robot_state(*move_group.getCurrentState());
//  robot_state.setToDefaultValues();
  Eigen::Affine3d pose_eigen;
  tf::poseMsgToEigen(pose, pose_eigen);
  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++){
      printf("%lf  ", end_effector_state(i,j));
    }
    printf("\n");
  }
  printf("\n");
  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++){
      if( i==1 && j==0 )
        pose_eigen(i,j) = end_effector_state(i,j) + 0.00000000001;
      printf("%lf  ", pose_eigen(i,j));
    }
    printf("\n");
  }
//  pose_eigen.translation() << end_effector_state.translation().x(), end_effector_state.translation().y(), end_effector_state.translation().z();
//  pose_eigen.rotation() << end_effector_state.rotation();
//  pose_eigen = end_effector_state;
  ROS_INFO_STREAM("Translation 2: " << pose_eigen.translation());
  ROS_INFO_STREAM("Rotation 2: " << pose_eigen.rotation());*/
  
  robot_state::RobotState robot_state(*move_group.getCurrentState());
//  robot_state.setToDefaultValues();
  if( !robot_state.setFromIK( joint_model_group
    , pose, end_effector_name, 5, 0.1) ){
    mythrow("cControl::get_robot_state : setFromIK failed\n");
  }
  return robot_state;
}

bool cControl::is_valid_pose(const geometry_msgs::Pose &pose){
  try{
    robot_state::RobotState state = get_robot_state(pose);
    printf("check valid state\n");
    print_joints(&state);
  }
  catch(int err){
    return false;
  }
  return true;
}


void cControl::print_pose(const geometry_msgs::Pose &pose){
  printf(" xyz = [%.3lf, %.3lf, %.3lf], xyzw : [%.3lf, %.3lf, %.3lf, %.3lf]\n"
      , pose.position.x, pose.position.y, pose.position.z
      , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

void cControl::print_joints(robot_state::RobotState *state){
  const robot_state::JointModelGroup *joint_model_group = state->getJointModelGroup(group_name);
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
  std::vector<double> joint_values;
  state->copyJointGroupPositions(joint_model_group, joint_values);
  printf("name : ");
  for(std::size_t i = 0; i < joint_names.size(); ++i){
    printf(" %s", joint_names[i].c_str());
  }
  printf("\nangle : ");
  for(std::size_t i = 0; i < joint_values.size(); ++i){
    printf(" %.3lf", joint_values[i]);
  }
  printf("\n");
}

