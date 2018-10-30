#include "ros/ros.h"
#include "cobot_sample_controller/cControl.h"
#include "traj_controller.h"
#include "math.h"

std::string result_dir;

bool create_trajectory(cControl &control, bool direction);
void print_trajectory(cControl &control, const std::string &file_name);
bool move_trajectory(cControl &control);

bool reach_angle(double q1, double q2);
double distance_point(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2);
void print_info(int wp, const geometry_msgs::Pose &p, const geometry_msgs::Pose &cr);
geometry_msgs::Pose extend_waypoint(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2, double l);

void print_move_cartesian(FILE *fp, double t_wp, const geometry_msgs::Point &wp, const std::vector<double> &wp_velo, double t_mv, const geometry_msgs::Point &mv, const std::vector<double> &mv_velo);
void print_move_joint_err(FILE *fp, double t, const std::vector<double> &err, const std::vector<double> &sum);
void print_move_joint_1(FILE *fp, double t, const std::vector<double> &p_wp, const std::vector<double> &v_wp, const std::vector<double> &p_jnt, const std::vector<double> &v_jnt);
void print_move_joint_2(FILE *fp, const std::vector<double> &v_cal);

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
		bool direction = false, b_move_traj = true;
    while( create_trajectory(control, direction) ) {
      print_trajectory(control, result_dir + "traj_original.txt");
      control.replan_velocity( 0.07, 0.3);
      print_trajectory(control, result_dir + "traj_const_velo.txt");
      b_move_traj = move_trajectory(control);
			ros::Duration(2).sleep();
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
	if(!direction) {
  	end_pose.position.y = start_pose.position.y+0.45;
	  end_pose.position.z = start_pose.position.z+0.03;
	}
	else {
  	end_pose.position.y = start_pose.position.y-0.45;
	  end_pose.position.z = start_pose.position.z-0.03;
	}

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

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// move_trajectory //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
bool move_trajectory(cControl &control) {
  printf("move_trajectory\n");
  FILE *fp1 = fopen( (result_dir + "move_cartesian.txt").c_str() , "wt");
  FILE *fp2 = fopen( (result_dir + "move_joint.txt").c_str() , "wt");
  FILE *fp3 = fopen( (result_dir + "move_joint_err.txt").c_str() , "wt");
  const trajectory_msgs::JointTrajectory &traj = control.get_trajectory();

  char target_str[512];

  ros::Rate r(100);
  ros::Time t_start = ros::Time::now(), t_print = t_start, start_time, read_time, run_time;
  
  // set acc to 3.14 rad/sec^2
  std::vector<double> acc(6), vel(6);
  for(int i=0;i<acc.size();i++) {
    acc[i] = M_PI;
		vel[i] = 0.0;
	}
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

	control.set_p_gain(joint_state.name[5], 1);
	control.set_i_gain(joint_state.name[5], 1000);

/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Loop  ////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
  bool stop = false, b_jnt_state = false;
	double dis_min, dt, limit_vel = M_PI / 3.0, sum, d_lab, d_time_move, kp=10, ki;
	ki = (kp*kp)/4;
	int j, k, count = 0;
	std::vector<std::vector<double>> sum_err;
	std::vector<double> old_err(6);
	for(int i=0; i<old_err.size(); i++)
		old_err[i] = 0.0;

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
		
		ROS_INFO("A");
    if( distance_point(current_pose, wp_pose) < 0.005 ) {
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

		ROS_INFO("B");
		dis_min = 999.999;
		dt = p.time_from_start.toSec() - d_time_move;
		//ROS_INFO("dt, p.time, read-start | %lf, %lf, %lf", dt, p.time_from_start.toSec(), (read_time - start_time).toSec());
		if(dt < 0.2) {
			ROS_WARN("dt : %lf", dt);
			dt = 0.2;
		}

		print_move_joint_1(fp2, d_time_move, p.positions, wp_j_velo, joint_state.position, joint_state.velocity);

		ROS_INFO("C");
		std::vector<double> v_sum(6);
		for(int i=0; i<v_sum.size(); i++)
			v_sum[i] = 0.0;
		count++;
		for(j=0; j<vel.size(); j++) {
//			double s = fabs(fabs(p.positions[j])-fabs(joint_state.position[j]));
//			double u = fabs(joint_state.velocity[j]);
			double err = p.positions[j] - joint_state.position[j];
//			double err = p.velocities[j] - joint_state.velocity[j];
			v_sum[j] += ((old_err[j]+err)/2)*dt;
			double sum = 0.0;
			for(int l=0; i<sum_err.size(); l++)
				sum += sum_err[i][j];

//			vel[j] = p.velocities[j] + (kp*err) + (ki*sum);
//			vel[j] = p.velocities[j] + (kp*err) + (ki*sum_err[j]/count);
			p.velocities[j] = ( p.positions[j] - joint_state.position[j] ) / dt;
			old_err[j] = err;

			if( vel[j] > limit_vel )	vel[j] = limit_vel;
			else if( vel[j] < -limit_vel )	vel[j] = -limit_vel;
		}
		ROS_INFO("D");

		sum_err.push_back(v_sum);
		if(sum_err.size() > 5)
			sum_err.erase(sum_err.begin());

		for(int i=0; i<vel.size(); i++) {
			if( p.positions[i] > joint_state.position[i] )
				vel[i] = vel[i] >= 0.0 ? vel[i] : -vel[i];
			else
				vel[i] = vel[i] >= 0.0 ? -vel[i] : vel[i];
		}

		ROS_INFO("E : v_sum.size() : %d, sum_err.size() : %d|%d", v_sum.size(), sum_err.size(), sum_err[0].size());
		for(int j=0; i<v_sum.size(); j++) {
			v_sum[j] = 0.0;
			for(int i=0; i<sum_err.size(); i++)
				v_sum[j] += sum_err[i][j];
		}

		ROS_INFO("F");
		print_move_joint_err(fp3, d_time_move, old_err, v_sum);
		ROS_INFO("G");
		print_move_joint_2(fp2, vel);

		ROS_INFO("H");
		for(j=0; j<vel.size(); j++)
		  vel[j] = 0.0;
		vel[vel.size()-1] = 1.5;


    control.move_velo(vel);
//    control.move_velo(p.velocities);
//    control.move_velo_acc(p.velocities, acc);
		sum = 0.0;
		do {
		  if( !control.wait_new_joint_state(&j_state, 1.0) ) {
				b_jnt_state = true;
				goto LB_EXIT_MOVE; // exit program if the new state does not come
			}
			for(k=0; k<joint_state.position.size(); k++)
				sum += fabs( joint_state.position[k] - j_state.position[k] );
		} while(sum < 0.0001);
		if(first_time) {
			start_time = ros::Time::now();
			first_time = false;
		}
  }

LB_EXIT_MOVE:
	if(!b_jnt_state) {
		for(j=0; j<vel.size(); j++)
		  vel[j] = 0.0;
		control.move_velo(vel);
	}
  if( fp1 )
    fclose(fp1);
  if( fp2 )
    fclose(fp2);
  if( fp3 )
    fclose(fp3);
  printf("move_traj end\n");

	return !b_jnt_state;
}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
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

void print_move_joint_err(FILE *fp, double t, const std::vector<double> &err, const std::vector<double> &sum)
{
	fprintf(fp, "%lf", t);
	for(int i=0; i<err.size(); i++)
		fprintf(fp, "  %lf", err[i]);
	for(int i=0; i<sum.size(); i++)
		fprintf(fp, "  %lf", sum[i]);
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

