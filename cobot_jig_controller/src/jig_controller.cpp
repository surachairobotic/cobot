
#include "ros/ros.h"
#include "cobot_planner/cControl.h"

std::string result_dir;

bool create_trajectory(cControl &control);
void print_trajectory(cControl &control, const std::string &file_name);
void move_trajectory(cControl &control);
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_jig_controller");
  cControl control("arm", "tool0");

  try{
    control.init();
    {
      ros::NodeHandle nh("~");
      std::string dir;
      nh.getParam("result_dir", dir);
      nh.deleteParam("result_dir");
      if( dir.size()==0 ){
        mythrow("No result_dir found\n");
      }
      result_dir = dir;
      if( dir[dir.size()-1]!='/' )
        result_dir+= '/';
    }
    if( create_trajectory(control) ){
      print_trajectory(control, result_dir + "traj_original.txt");
      control.replan_velocity( 0.05, 0.3);
      print_trajectory(control, result_dir + "traj_const_velo.txt");
      move_trajectory(control);
    }
  }
  catch(const std::string &err){
    ROS_ERROR("%s", err.c_str());
  }
  printf("end\n");
  ros::shutdown();
  return 0;
}


bool create_trajectory(cControl &control){
  geometry_msgs::Pose start_pose = control.get_current_pose()
    , end_pose = start_pose;

//  start_pose.position.x+= 0.1;

  end_pose.position.x = start_pose.position.x;
  end_pose.position.y = start_pose.position.y+0.45;
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

  bool b = control.plan_line(start_pose, end_pose, 0.05);
//  bool b = control.plan_p2p(start_pose, end_pose);
  if( b ){
    printf("Trajectory has been created successfully.\n\n");
  }
  else{
    ROS_ERROR("Trajectory was not created successfully.\n\n");
  }
  return b;
}


void print_trajectory(cControl &control, const std::string &file_name){
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  FILE *fp = fopen(file_name.c_str(), "wt");
  if( !fp ){
    ROS_ERROR("Cannot create file result file : %s", file_name.c_str());
  }
  for(int i=0;i<traj.joint_names.size();i++){
    printf("joint : %s\n", traj.joint_names[i].c_str());
  }
  for(int i=0;i<traj.points.size();i++){
    const trajectory_msgs::JointTrajectoryPoint &p = traj.points[i];
    geometry_msgs::Pose pose;
    std::vector<double> velo;
    control.get_cartesian_position(p.positions, pose);
    control.get_cartesian_velocity(p.positions, p.velocities, velo );
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
      geometry_msgs::Pose pose;
      control.get_cartesian_position(p.positions, pose);
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
  printf("move_trajectory\n");
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  FILE *fp = fopen( (result_dir + "move_traj.txt").c_str() , "wt");
  FILE *fp2 = fopen( (result_dir + "log_jig_control.txt").c_str() , "wt");
  if( !fp ){
    ROS_ERROR("Cannot create move_traj file");
  }
  if( !fp2 ){
    ROS_ERROR("Cannot create log_jig_control file");
  }
  ros::Rate r(100);
  ros::Time t_start = ros::Time::now(), t_print = t_start;

  { // set acc to 3.14 rad/sec^2
    std::vector<double> acc(6);
    for(int i=0;i<acc.size();i++)
      acc[i] = M_PI;
    control.set_acc(acc);
  }

  for(int i=1;i<traj.points.size();i++){
    const trajectory_msgs::JointTrajectoryPoint &p = traj.points[i];
    geometry_msgs::Pose pose;
    geometry_msgs::Pose current_pose;
    char target_str[512];
    std::vector<double> new_velo;
    std::vector<double> stop_velo;
    new_velo=p.velocities;
    stop_velo=p.velocities;

    control.get_cartesian_position(p.positions, pose);
    // show target joint angle and velocity
    printf("waypoint %d\njoints : ", i);
    printf("\nangle : ");
    for(int j=0;j<p.positions.size();j++)
      printf(" %.3lf", p.positions[j]);
    printf("\nvelo : ");
    for(int j=0;j<p.velocities.size();j++)
      printf(" %.3lf", p.velocities[j]);
    printf("\n");

    fprintf(fp2, "waypoint %d\njoints : ", i);
    fprintf(fp2, "\nangle : ");
    for(int j=0;j<p.positions.size();j++)
      fprintf(fp2, " %.3lf", p.positions[j]);
    fprintf(fp2, "\nvelo : ");
    for(int j=0;j<p.velocities.size();j++)
      fprintf(fp2, " %.3lf", p.velocities[j]);
    fprintf(fp2, "\n");

    // write to file
    if(fp){
      // debug sprintf(target_str +strlen(target_str), "|A|");
      target_str[0] = 0;
      for(int j=0;j<p.positions.size();j++){
        sprintf(target_str +strlen(target_str), " %lf", p.positions[j]); // target joint angle
      }
      for(int j=0;j<p.velocities.size();j++){
        sprintf(target_str +strlen(target_str), " %lf", p.velocities[j]); // target joint velocity
      }
      for(int j=0;j<p.accelerations.size();j++){
        sprintf(target_str +strlen(target_str), " %lf", p.accelerations[j]); // target joint acceleration
      }
      // target position in xyz and quaternion
      sprintf(target_str + strlen(target_str), " %lf %lf %lf %lf %lf %lf %lf"
        , pose.position.x, pose.position.y, pose.position.z
        , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }


// //    control.move_velo(p.velocities);
    // control.move_pos_velo(p.positions, p.velocities);
//    control.move_pos_velo(p.positions, new_velo);

    if (i < traj.points.size()-1){
//    	for(int j=0;j<p.velocities.size();j++){
//    	        if (p.velocities[j]== 0){
//    	        	p.positions[j]= 0.0001;
//    	        }
    	control.move_velo(p.velocities);
    	printf("vel control\n");
    	fprintf(fp2, "vel control\n");
    }
    else{
    	for(int j=0;j<new_velo.size();j++){
    		if (new_velo[j]== 0){
    			new_velo[j]= 0.0001;
    		}
		control.move_pos_velo(p.positions, new_velo);
	}
    	printf("pos control\n");
    	fprintf(fp2, "pos control\n");
    }
    bool b_reach;
    ros::Duration dt[10];

    do{
      dt[0] = ros::Time::now() - t_start;
      r.sleep();
      dt[1] = ros::Time::now() - t_start;
//      const std::vector<double> cur_joints = control.get_current_joints(); // get only joints'position
      sensor_msgs::JointState joint_state;
      // get joints'position, velocity, load
      if( !control.wait_new_joint_state(&joint_state, 1.0) ){
        goto LB_EXIT_MOVE;  // exit program if the new state does not come within 0.2 seconds
      }
      dt[2] = ros::Time::now() - t_start;
      b_reach = true;


/*      for(int j=2;j>=0;j--){
  //  for(int j=cur_joints.size()-1;j>=0;j--){
        if( !reach_angle(joint_state.position[j], p.positions[j]) ){ // check if reached the target angles
          b_reach = false;
          break;
        }
      }
*/

      control.get_cartesian_position(joint_state.position, current_pose);
      double check_dis = sqrt((current_pose.position.x-pose.position.x)*(current_pose.position.x-pose.position.x)
    		  + (current_pose.position.y-pose.position.y)*(current_pose.position.y-pose.position.y)
    		  + (current_pose.position.z-pose.position.z)*(current_pose.position.z-pose.position.z));
      if(check_dis>0.01){
    	  b_reach = false;
      }

      printf("current_pose : %lf %lf %lf\n", current_pose.position.x, current_pose.position.y, current_pose.position.z);
      fprintf(fp2, "current_pose : %lf %lf %lf\n", current_pose.position.x, current_pose.position.y, current_pose.position.z);

      dt[3] = ros::Time::now() - t_start;
      ros::Time t = ros::Time::now();
      // write to file
      if( fp ){
          // debug fprintf(fp, "|B|");
        geometry_msgs::Pose pose2;
        control.get_cartesian_position(joint_state.position, pose2);
        fprintf(fp, "%lf", (t-t_start).toSec());
        for(int j=0;j<joint_state.position.size();j++){
          fprintf(fp, " %lf", joint_state.position[j]); // current joint angle
        }
        for(int j=0;j<joint_state.position.size();j++){
          fprintf(fp, " %lf", joint_state.velocity[j]); // current joint velocity
        }
        for(int j=0;j<joint_state.position.size();j++){
          fprintf(fp, " %lf", joint_state.effort[j]); // current joint load
        }
        fprintf(fp, " %lf %lf %lf %lf %lf %lf %lf%s"
          , pose2.position.x, pose2.position.y, pose2.position.z  // current pose in xyz and quaternion
          , pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w
          , target_str);

        for(int j=0;j<joint_state.position.size();j++){
                  fprintf(fp, " %lf", joint_state.effort[j]); // current joint load
        }
        fprintf(fp, "|WP_%d|", i);
        fprintf(fp, "\n");
      } // if
      dt[4] = ros::Time::now() - t_start;
      if( (t-t_print).toSec() > 1.0 ){
        for(int j=0;j<joint_state.position.size();j++){
          // show (current position, velocity load) and (target position, velocity)
          printf("joint [%d] : position = %lf, velocity %lf, load = %lf, target_position = %lf, target_velocity = %lf\n"
            , j, joint_state.position[j], joint_state.velocity[j], joint_state.effort[j]
            , p.positions[j], p.velocities[j]);
          printf("joint [%d] : new velocity = %lf\n", j, new_velo[j]);
          fprintf(fp2, "joint [%d] : position = %lf, velocity %lf, load = %lf, target_position = %lf, target_velocity = %lf\n"
            , j, joint_state.position[j], joint_state.velocity[j], joint_state.effort[j]
            , p.positions[j], p.velocities[j]);
          fprintf(fp2, "joint [%d] : new velocity = %lf\n", j, new_velo[j]);
        }
        t_print = t;

        printf("time : %lf, %lf, %lf, %lf, %lf, %d\n", dt[0].toSec(), dt[1].toSec(), dt[2].toSec(), dt[3].toSec(), dt[4].toSec(), i); // show calculation time
        fprintf(fp2, "time : %lf, %lf, %lf, %lf, %lf, %d\n", dt[0].toSec(), dt[1].toSec(), dt[2].toSec(), dt[3].toSec(), dt[4].toSec(), i); // show calculation time
      } // if

      if( !ros::ok() )
        return;
    } // do
    while(!b_reach);
    if (i == traj.points.size()-1){
    	for(int j=0;j<p.velocities.size();j++){
    		stop_velo[j]= 0;
    	}
    	control.move_velo(stop_velo);
    } // if
  } // for
  printf("move_traj end\n");
  fprintf(fp2, "move_traj end\n");
LB_EXIT_MOVE:
  if( fp ){
    fclose(fp);
  }
  if( fp2 ){
    fclose(fp2);
  }
}
