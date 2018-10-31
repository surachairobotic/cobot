#include "ros/ros.h"
#include "cobot_sample_controller/cControl.h"
#include "traj_controller.h"
#include "math.h"

std::string result_dir;

bool create_trajectory(cControl &control, bool direction);
void print_trajectory(cControl &control, const std::string &file_name);
void move_trajectory(cControl &control);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cobot_sample_controller");
  cControl control("arm", "tool0");
//  TrajectoryController trajControl;
  try {
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
		bool direction = false;
    while( create_trajectory(control, direction) ) {
      print_trajectory(control, result_dir + "traj_original.txt");
      control.replan_velocity( 0.1, 0.3);
      print_trajectory(control, result_dir + "traj_const_velo.txt");
      move_trajectory(control);
			ros::Duration(5).sleep();
			direction = !direction;
			//trajControl.actionCallback();
    }
  }
  catch(const std::string &err) {
    ROS_ERROR("%s", err.c_str());
  }
  printf("end\n");
  ros::shutdown();
  return 0;
}

bool create_trajectory(cControl &control, bool direction) {
  geometry_msgs::Pose start_pose = control.get_current_pose(), end_pose = start_pose;

//  start_pose.position.x+= 0.1;

  end_pose.position.x = start_pose.position.x;
	if(!direction)
  	end_pose.position.y = start_pose.position.y+0.45;
	else
  	end_pose.position.y = start_pose.position.y-0.45;
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
  
  bool b = control.plan_line(start_pose, end_pose, 0.01);
//  bool b = control.plan_p2p(start_pose, end_pose);
  if( b )
    printf("Trajectory has been created successfully.\n\n");
  else
    ROS_ERROR("Trajectory was not created successfully.\n\n");

  return b;
}

void print_trajectory(cControl &control, const std::string &file_name) {
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  FILE *fp = fopen(file_name.c_str(), "wt");
  if( !fp )
    ROS_ERROR("Cannot create file result file : %s", file_name.c_str());

  for(int i=0; i<traj.joint_names.size(); i++)
    printf("joint : %s\n", traj.joint_names[i].c_str());

  for(int i=0; i<traj.points.size(); i++) {
    const trajectory_msgs::JointTrajectoryPoint &p = traj.points[i];
    geometry_msgs::Pose pose;
    std::vector<double> velo;
    control.get_cartesian_position(p.positions, pose);
    control.get_cartesian_velocity(p.positions, p.velocities, velo );
    printf("point [%d]  time : %lf\n", i, p.time_from_start.toSec() );
    printf("angle :");
    if( fp )
        fprintf(fp, "%lf", p.time_from_start.toSec());
    for(int j=0; j<p.positions.size(); j++) {
      printf(" %.3lf", p.positions[j]);
      if( fp )
        fprintf(fp, " %lf", p.positions[j]);
    }
    printf("\nangular velo :");
    for(int j=0; j<p.velocities.size(); j++) {
      printf(" %.3lf", p.velocities[j]);
      if( fp )
        fprintf(fp, " %lf", p.velocities[j]);
    }
    printf("\nangular acc :");
    for(int j=0; j<p.accelerations.size(); j++) {
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
    printf("quart : %.3lf %.3lf %.3lf %.3lf\n", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    printf("velo : ");
    for(int i=0; i<velo.size(); i++)
      printf(" %.3lf", velo[i]);
    printf("\n");
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
    
  printf("\nstart - end point\n");
  const int ii[] = { 0, (int)traj.points.size()-1 };
  for(int i=0; i<2; i++){
    const trajectory_msgs::JointTrajectoryPoint &p = traj.points[ii[i]];
    geometry_msgs::Pose pose;
    control.get_cartesian_position(p.positions, pose);
    printf("xyz : %.3lf %.3lf %.3lf, w : %.3lf, %.3lf, %.3lf, %.3lf\n"
      , pose.position.x, pose.position.y, pose.position.z
      , pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }
}

bool reach_angle(double q1, double q2) {
  const double PI2 = 2 * M_PI;
  while(q1 - q2 > PI2)
    q2+= PI2;
  while(q2 - q1 > PI2)
    q1+= PI2;
  return fabs(q1-q2) < 0.01;
}

double distance_point(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2) {
  return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x)
	    + (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y)
	    + (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
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

geometry_msgs::Pose extend_waypoint(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, double l) {
  double dx = p2.position.x - p1.position.x, dy = p2.position.y - p1.position.y, dz = p2.position.z - p1.position.z;
  double dis = sqrt( pow(dx, 2) + pow(dy, 2) + pow(dz, 2) );
  geometry_msgs::Pose p;
  p.position.x = dx*l/dis + p1.position.x;
  p.position.y = dy*l/dis + p1.position.y;
  p.position.z = dz*l/dis + p1.position.z;
  p.orientation = p2.orientation;
  return p;
}

void move_trajectory(cControl &control) {
  printf("move_trajectory\n");
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();
  FILE *fp = fopen( (result_dir + "move_traj.txt").c_str() , "wt");
  FILE *fp2 = fopen( (result_dir + "log_jig_control.txt").c_str() , "wt");
  FILE *fp3 = fopen( (result_dir + "move_vel_xyz.txt").c_str() , "wt");

  char target_str[512];
  ros::Duration dt[10];

  if( !fp )
    ROS_ERROR("Cannot create move_traj file");
  if( !fp2 )
    ROS_ERROR("Cannot create log_jig_control file");
  if( !fp3 )
    ROS_ERROR("Cannot create move_vel_xyz file");

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
/*  for(int i=0;i<acc.size();i++) {
    ROS_INFO("p.acc[%d] : %lf", i, fabs(p.accelerations[i]));
		acc[i] = fabs(p.accelerations[i]);
	}
  control.set_acc(acc);
*/
  geometry_msgs::Pose wp_pose, current_pose, ex_pose, pose, tmp_pose;
  ros::Time t = ros::Time::now();
  if( !control.wait_new_joint_state(&joint_state, 1.0) ) return; // exit program if the new state does not come
  control.get_cartesian_position(joint_state.position, current_pose);
  control.get_cartesian_position(p.positions, wp_pose);
  std::vector<double> current_joint = control.get_current_joints();
	ROS_INFO("joint_state_stamp : %lf", joint_state.header.stamp.sec);  

  // write to file
  if(fp){
    // debug sprintf(target_str +strlen(target_str), "|A|");
    target_str[0] = 0;
    for(int j=0;j<p.positions.size();j++)
      sprintf(target_str +strlen(target_str), " %lf", p.positions[j]); // target joint angle
    for(int j=0;j<p.velocities.size();j++)
      sprintf(target_str +strlen(target_str), " %lf", p.velocities[j]); // target joint velocity
    for(int j=0;j<p.accelerations.size();j++)
      sprintf(target_str +strlen(target_str), " %lf", p.accelerations[j]); // target joint acceleration
    // target position in xyz and quaternion
    sprintf(target_str + strlen(target_str), " %lf %lf %lf %lf %lf %lf %lf"
            , wp_pose.position.x, wp_pose.position.y, wp_pose.position.z
            , wp_pose.orientation.x, wp_pose.orientation.y, wp_pose.orientation.z, wp_pose.orientation.w);
  }
//  ex_pose = extend_waypoint(current_pose, wp_pose, 0.03);
//  control.get_joint_positions(ex_pose, p.positions);

  if(fp2) {
    fprintf(fp2, "[WP:%d]", i);
    for(int j=0; j<p.positions.size(); j++)
      fprintf(fp2, "(%.3lf:%.3lf)",p.positions[j], p.velocities[j]);
    fprintf(fp2, "\n[CP:%d]", i);
    for(int j=0; j<joint_state.position.size(); j++)
      fprintf(fp2, "(%.3lf:%.3lf)",joint_state.position[j], joint_state.velocity[j]);
    fprintf(fp2, "\n");
  }

////////////////////////////////////////////////////////////////////////////////////////////
  bool stop = false;
  while( !stop && ros::ok() ) {
    control.get_cartesian_position(p.positions, tmp_pose);
    print_info(i, tmp_pose, current_pose);
		
      // write to file
      if( fp ){
          // debug fprintf(fp, "|B|");
        geometry_msgs::Pose pose2;
				control.get_cartesian_position(joint_state.position, pose2);
        fprintf(fp, "%lf", (t-t_start).toSec());
        for(int j=0;j<joint_state.position.size();j++)
          fprintf(fp, " %lf", joint_state.position[j]); // current joint angle
        for(int j=0;j<joint_state.position.size();j++)
          fprintf(fp, " %lf", joint_state.velocity[j]); // current joint velocity
        for(int j=0;j<joint_state.position.size();j++)
          fprintf(fp, " %lf", joint_state.effort[j]); // current joint load
        fprintf(fp, " %lf %lf %lf %lf %lf %lf %lf%s"
                  , pose2.position.x, pose2.position.y, pose2.position.z  // current pose in xyz and quaternion
                  , pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w
                  , target_str);
        for(int j=0; j<joint_state.position.size();j++)
                  fprintf(fp, " %lf", joint_state.effort[j]); // current joint load
        fprintf(fp, "|WP_%d|", i);
        fprintf(fp, "\n");
        dt[0] = ros::Time::now() - t;
        t = ros::Time::now();
      } // if

/*	  for(int j=0; j<p.velocities.size(); j++)
	    if( fabs(p.velocities[j]) < 0.001 ) {
		    p.velocities[j] = 0.001;
		    if( p.velocities[j] < 0 ) p.velocities[j] = p.velocities[j] * -1;
		  }
*/
//    control.move_pos_velo(p.positions, p.velocities);
//    control.move_velo(p.velocities);
    if( !control.wait_new_joint_state(&joint_state, 1.0) ) goto LB_EXIT_MOVE; // exit program if the new state does not come
//    if( !control.get_last_joint_state(&joint_state) ) goto LB_EXIT_MOVE; // exit program if the new state does not come

		read_time = ros::Time::now();

		if(fp3) {
			std::vector<double> xyz_vel;
			control.get_cartesian_velocity(joint_state.position, joint_state.velocity, xyz_vel);
			for(int j=0; j<xyz_vel.size(); j++)
				fprintf(fp3, " %lf", xyz_vel[j]);
			fprintf(fp3, "\n");
			ROS_INFO("xyz_vel.size() : %d", xyz_vel.size());
		}

    control.get_cartesian_position(joint_state.position, current_pose);
		ROS_INFO("joint_state_stamp : %d", joint_state.header.stamp.sec);  

    if(fp2) {
      fprintf(fp2, "[WP:%d]", i);
      for(int j=0; j<p.positions.size(); j++)
        fprintf(fp2, "(%.3lf:%.3lf)",p.positions[j], p.velocities[j]);
      fprintf(fp2, "\n[CP:%d]", i);
      for(int j=0; j<joint_state.position.size(); j++)
        fprintf(fp2, "(%.3lf:%.3lf)",joint_state.position[j], joint_state.velocity[j]);
      fprintf(fp2, "\n");
			fprintf(fp2, "joint_state_stamp : %d\n", joint_state.header.stamp.sec);
    }

    printf("dist : %lf\n", distance_point(current_pose, wp_pose));
    if( distance_point(current_pose, wp_pose) < 0.005 ) {
      if( i < traj.points.size()-1 ) {
        i++;
        p = traj.points[i];
				/*
				for(int i=0;i<acc.size();i++) {
					ROS_INFO("p.acc[%d] : %lf", i, fabs(p.accelerations[i]));
					acc[i] = fabs(p.accelerations[i]);
				}
				control.set_acc(acc);
				*/
        control.get_cartesian_position(p.positions, wp_pose);

        // write to file
        if(fp){
          // debug sprintf(target_str +strlen(target_str), "|A|");
          target_str[0] = 0;
          for(int j=0;j<p.positions.size();j++)
            sprintf(target_str +strlen(target_str), " %lf", p.positions[j]); // target joint angle
          for(int j=0;j<p.velocities.size();j++)
            sprintf(target_str +strlen(target_str), " %lf", p.velocities[j]); // target joint velocity
          for(int j=0;j<p.accelerations.size();j++)
            sprintf(target_str +strlen(target_str), " %lf", p.accelerations[j]); // target joint acceleration
          // target position in xyz and quaternion
          sprintf(target_str + strlen(target_str), " %lf %lf %lf %lf %lf %lf %lf"
                  , wp_pose.position.x, wp_pose.position.y, wp_pose.position.z
                  , wp_pose.orientation.x, wp_pose.orientation.y, wp_pose.orientation.z, wp_pose.orientation.w);
        }
//        ex_pose = extend_waypoint(current_pose, wp_pose, 0.03);
				//ROS_INFO("ex[%lf, %lf, %lf, %lf, %lf, %lf");
//        control.get_joint_positions(ex_pose, p.positions);
      }
      else
        stop = true;
    }
		/*else */{
			if(first_time) start_time = read_time;
			double dis_min = 999.999, dt = (p.time_from_start - (read_time - start_time)).toSec();
			ROS_INFO("dt, p.time, read-start | %lf, %lf, %lf", dt, p.time_from_start.toSec(), (read_time - start_time).toSec());
//			int indx = 0;
//			std::vector<double> dis_t;
//			for(int i=0; i<p.positions.size(); i++) {
//				dis_t.push_back( fabs( p.positions[i] - joint_state.position[i] ) / joint_state.velocity[i] );
//				if(dis_t[i] <= dis_min && dis_t[i] > 0.01) {	dis_min = dis_t[i];  indx = i;  }
//			}
			if(dt < 0.02) dt = 0.02;

			double limit_vel = M_PI / 2.0;
			for(int j=0; j<p.velocities.size(); j++) {
				p.velocities[j] = ( p.positions[j] - joint_state.position[j] ) / dt;
				if( p.velocities[j] > limit_vel )	p.velocities[j] = limit_vel;
				else if( p.velocities[j] < -limit_vel )	p.velocities[j] = -limit_vel;
			}
//			for(int i=0;i<acc.size();i++)
//				ROS_INFO("p.acc[%d] : %lf", i, p.accelerations[i]);

	    //control.move_velo(p.velocities);
		}
		for(int j=0; j<p.velocities.size(); j++)
			ROS_INFO("p.velocities[%d] : %lf", j, p.velocities[j]);

    control.move_velo(p.velocities);
		double sum = 0.0;
		do {
		  if( !control.wait_new_joint_state(&j_state, 1.0) ) goto LB_EXIT_MOVE;
			for(int k=0; k<joint_state.position.size(); k++)
				sum += fabs( joint_state.position[k] - j_state.position[k] );
			ROS_ERROR("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");
		} while(sum < 0.0001);
		if(first_time) {
			start_time = ros::Time::now();
			first_time = false;
		}
  }

  for(int j=0; j<p.velocities.size(); j++)
    p.velocities[j] = 0.0;
  control.move_velo(p.velocities);
  
  printf("move_traj end\n");
  fprintf(fp2, "move_traj end\n");

LB_EXIT_MOVE:
  if( fp )
    fclose(fp);
  if( fp2 )
    fclose(fp2);
  if( fp3 )
    fclose(fp3);
}
