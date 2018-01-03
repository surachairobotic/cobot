
#include "ros/ros.h"
#include "cobot_sample_controller/cControl.h"


bool create_trajectory(cControl &control);
void print_trajectory(cControl &control, const char *file_name);
void move_trajectory(cControl &control);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_sample_controller");
  cControl control("arm", "tool0");
  control.init();

  try{
    if( create_trajectory(control) ){
      print_trajectory(control, "/home/tong/catkin_ws/temp/traj_original.txt");
      control.replan_velocity( 0.05, 0.3);
      print_trajectory(control, "/home/tong/catkin_ws/temp/traj_const_velo.txt");
      move_trajectory(control);

/*      {
        const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
        printf("\nstart - end point\n");
        const int ii[] = { 0, (int)traj.points.size()-1 };
        for(int i=0;i<2;i++){
          const trajectory_msgs::JointTrajectoryPoint &p = traj.points[ii[i]];
          const geometry_msgs::Pose pose = control.get_cartesian_position(p.positions);
          printf("xyz : %.3lf %.3lf %.3lf, w : %.3lf, %.3lf, %.3lf, %.3lf\n"
            , pose.position.x, pose.position.y, pose.position.z
            , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
      }*/
    }
  }
  catch(int my_error){}
  ros::shutdown();
  return 0;
}


bool create_trajectory(cControl &control){
  geometry_msgs::Pose start_pose = control.get_current_pose()
    , end_pose = start_pose;

//  start_pose.position.x+= 0.1;

  end_pose.position.x = start_pose.position.x;
  end_pose.position.y = start_pose.position.y + 0.1;
  end_pose.position.z = start_pose.position.z;

  if( !control.is_valid_pose(start_pose) ){
    ROS_ERROR("Invalid start_pos : ");
    cControl::print_pose(start_pose);
    return false;
  }
  
  if( !control.is_valid_pose(end_pose) ){
    ROS_ERROR("Invalid end_pos : ");
    cControl::print_pose(end_pose);
    return false;
  }
  
  ROS_INFO("start pose : ");
  cControl::print_pose(start_pose);
  ROS_INFO("end pose : ");
  cControl::print_pose(end_pose);

  bool b = control.plan_p2p(start_pose, end_pose);
  if( b ){
    printf("Trajectory has been created successfully.\n\n");
  }
  else{
    ROS_ERROR("Trajectory was not created successfully.\n\n");
  }
  return b;
}


void print_trajectory(cControl &control, const char *file_name){
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  FILE *fp = fopen(file_name, "wt");
  if( !fp ){
    ROS_ERROR("Cannot create file 'traj.txt'\n");
  }
  for(int i=0;i<traj.joint_names.size();i++){
    printf("joint : %s\n", traj.joint_names[i].c_str());
  }
  for(int i=0;i<traj.points.size();i++){
    const trajectory_msgs::JointTrajectoryPoint &p = traj.points[i];
    const geometry_msgs::Pose pose = control.get_cartesian_position(p.positions);
    const std::vector<double> velo = control.get_cartesian_velocity(p.positions, p.velocities );
    printf("point [%d]  time : %lf\n", i, p.time_from_start.toSec() );
    printf("angle :");
    if( fp )
        fprintf(fp, "%lf", p.time_from_start.toSec());
    for(int j=0;j<p.positions.size();j++){
      printf(" %.3lf", p.positions[j]);
      if( fp )
        fprintf(fp, " %lf", p.positions[j]);
    }
    printf("\nangular velo :");
    for(int j=0;j<p.velocities.size();j++){
      printf(" %.3lf", p.velocities[j]);
      if( fp )
        fprintf(fp, " %lf", p.velocities[j]);
    }
    printf("\nangular acc :");
    for(int j=0;j<p.accelerations.size();j++){
      printf(" %.3lf", p.accelerations[j]);
      if( fp )
        fprintf(fp, " %lf", p.accelerations[j]);
    }
/*    printf("\neff :");
    for(int j=0;j<p.effort.size();j++){
      printf(" %.3lf", p.effort[j]);
      if( fp )
        fprintf(fp, " %lf", p.effort[j]);
    }*/
    printf("\nxyz : %.3lf %.3lf %.3lf\n", pose.position.x, pose.position.y, pose.position.z);
    printf("quart : %.3lf %.3lf %.3lf %.3lf\n"
      , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    printf("velo : ");
    for(int i=0;i<velo.size();i++)
      printf(" %.3lf", velo[i]);
    printf("\n");
    if( fp ){
      fprintf(fp, " %lf %lf %lf %lf %lf %lf %lf"
        , pose.position.x, pose.position.y, pose.position.z
        , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      for(int i=0;i<velo.size();i++)
        fprintf(fp, " %lf", velo[i]);
      fprintf(fp, "\n");
    }

  }
  if(fp)
    fclose(fp);
  {
    printf("\nstart - end point\n");
    const int ii[] = { 0, (int)traj.points.size()-1 };
    for(int i=0;i<2;i++){
      const trajectory_msgs::JointTrajectoryPoint &p = traj.points[ii[i]];
      const geometry_msgs::Pose pose = control.get_cartesian_position(p.positions);
      printf("xyz : %.3lf %.3lf %.3lf, w : %.3lf, %.3lf, %.3lf, %.3lf\n"
        , pose.position.x, pose.position.y, pose.position.z
        , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }
  }
}

bool reach_angle(double q1, double q2){
  const double PI2 = 2 * M_PI;
  while(q1 - q2 > PI2)
    q2+= PI2;
  while(q2 - q1 > PI2)
    q1+= PI2;
  return fabs(q1-q2) < 0.01;
}

void move_trajectory(cControl &control){
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  ros::Rate r(100);
  ros::Time t_start = ros::Time::now();
  for(int i=1;i<traj.points.size();i++){
    const trajectory_msgs::JointTrajectoryPoint &p = traj.points[i];
    printf("waypoint %d\njoints : ", i);
    printf("\nangle : ");
    for(int j=0;j<p.positions.size();j++)
      printf(" %.3lf", p.positions[j]);
    printf("\nvelo : ");
    for(int j=0;j<p.velocities.size();j++)
      printf(" %.3lf", p.velocities[j]);
    printf("\n");
    
//    control.move_velo(p.velocities);
    control.move_pos_velo(p.positions, p.velocities);
    bool b_reach;
    do{
      r.sleep();
      const std::vector<double> cur_joints = control.get_current_joints();
      b_reach = true;
      for(int j=0;j>=0;j--){
  //  for(int j=cur_joints.size()-1;j>=0;j--){
        if( !reach_angle(cur_joints[j], p.positions[j]) ){
          b_reach = false;
          break;
        }
      }
      if( !ros::ok() )
        return;
      ros::Time t = ros::Time::now();
      if( (t-t_start).toSec() > 1.0 ){
        for(int j=0;j<cur_joints.size();j++){
          printf("joint [%d] : cur = %lf, tar = %lf, vel = %lf\n", j, cur_joints[j], p.positions[j]
            , p.velocities[j]);
        }
        t_start = t;
      }
    }
    while(!b_reach);
  }
  printf("move_traj end\n");
}
