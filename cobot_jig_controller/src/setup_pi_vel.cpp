
#include "ros/ros.h"
#include "cobot_sample_controller/cControl.h"
#include "math.h"
#include <string>

std::string result_dir;

bool create_trajectory(cControl &control);
bool waitUntilStateChange(cControl &control, int indx, ros::Time &t);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setup_pi_vel");
  cControl control("arm", "tool0");
  FILE *fp;
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
//			result_dir += "pi_vel_150";
//      result_dir += '/';
    }
		if( !create_trajectory(control) )
			return -1;
		// set acc to 3.14 rad/sec^2
		std::vector<double> acc(6), vel(6), pos(6);
		for(int i=0;i<acc.size();i++) {
		  acc[i] = 2.0;
			vel[i] = 0.0;
			pos[i] = 0.0;
		}
		control.set_acc(acc);
		for(int j=0; j<vel.size(); j++)
			vel[j] = 0.0;
		sensor_msgs::JointState joint_state;
		bool exit = false;
		int sel_jnt = 5, P = 830, I = 40;
				control.set_p_gain("J6", P);
				control.set_i_gain("J6", I);
		//830
////////////////////////////////// velocity control to sin wave ::: V2

		double s = 5;
		double dt, bound = 1.25, m = (M_PI-(-M_PI))/(s-0), c = (-M_PI)-(m*0);
		bool state = false;
		ros::Time start_time, t_save, run_time;
		std::string f_name = (result_dir + "move_wave.txt");
		fp = fopen( f_name.c_str() , "wt");
		double count = 0;
////////////////
		std::vector<double> v_sum;
		double old_err=0.0, kp=0.01, ki = (kp*kp)/4, kd=0.05, target=0.0;
		if( !control.wait_new_joint_state(&joint_state, 1.0) ) return false;
////////////////
		start_time = t_save = run_time = ros::Time::now();
		while(count < 10) {
			run_time = ros::Time::now();
			dt = (run_time-start_time).toSec();
			if(dt > s) start_time = ros::Time::now();
//			target = cos(m*dt+c)*1.5;
////////////////
			target = cos(m*dt+c)*1.5;
			double err = (target*dt) - joint_state.position[sel_jnt];
			v_sum.push_back( ((old_err+err)/2)*dt );
			if(v_sum.size() > 10)
				v_sum.erase(v_sum.begin());
			double sum = 0.0;
			for(int l=0; l<v_sum.size(); l++)
				sum += v_sum[l];
			vel[sel_jnt] = target + (kp*err) + (ki*sum) + (kd*(old_err-err)/dt);
			old_err = err;
////////////////
			control.move_velo(vel);
			if( !control.wait_new_joint_state(&joint_state, 1.0) ) return false;
			else if(fp) {
				run_time = ros::Time::now();
				count = (run_time-t_save).toSec();
				fprintf(fp, "%lf %lf %lf\n", count, target, joint_state.velocity[sel_jnt]);
			}
			ROS_INFO("target, err, sum, (old_err-err)/dt : %lf, %lf, %lf, %lf", target, err, sum, (old_err-err)/dt);
		}
		vel[sel_jnt] = 0.0;
		control.move_velo(vel);
		if( fp )
			fclose(fp);

//////////////////////////////////
/*
//				control.set_goal_torque("J5", 20);

//		for(P=1010; P<=10000; P+=10)
//			for(I=0; I<=2000; I+=100)
			{
				ROS_INFO("P, I : %d, %d", P, I);//control.get_p_gain(), control.get_i_gain());
				ros::Time t_start, t_save, run_time;
				std::string f_name = (result_dir + "move_p" + std::to_string(P) + "_i" + std::to_string(I) + ".txt");
//				std::string f_name = (result_dir + "move_pid.txt");
				fp = fopen( f_name.c_str() , "wt");
				control.set_p_gain("J6", P);
				control.set_i_gain("J6", I);
std::vector<double> v_sum;
double old_err, limit_vel = M_PI*2, kp=10, ki = (kp*kp)/4, kd=0.5, dt, target=1.5;
		double s = 5, m = (M_PI-(-M_PI))/(s-0), c = (-M_PI)-(m*0);
				vel[sel_jnt] = target;
				control.move_velo(vel);
				if( !waitUntilStateChange(control, sel_jnt, t_start) ) return false;
				if(fp)
					fprintf(fp, "%lf %lf %lf\n", 0.00, target, 0.00);

				t_save = t_start;
				run_time = ros::Time::now();
				while( (run_time-t_save).toSec() < 30 && !exit) {
					if( !control.wait_new_joint_state(&joint_state, 1.0) ) {
						exit = true;
					}
					else if(fp) {
						fprintf(fp, "%lf %lf %lf\n", (run_time-t_save).toSec(), target, joint_state.velocity[sel_jnt]);
					}
			double dt = (run_time-t_start).toSec();
			if(dt > s) t_start = ros::Time::now();
			target = cos(m*dt+c)*1.5;
			ROS_INFO("dt, target = %lf:%lf", dt, target);
			double err = (target*(run_time-t_save).toSec()) - joint_state.position[sel_jnt];
			v_sum.push_back( ((old_err+err)/2)*dt );
		if(v_sum.size() > 10)
			v_sum.erase(v_sum.begin());

			double sum = 0.0;
			for(int l=0; l<v_sum.size(); l++)
				sum += v_sum[l];

			vel[sel_jnt] = target + (kp*err) + (ki*sum) + (kd*(old_err-err)/dt);
			old_err = err;

			if( vel[sel_jnt] > limit_vel )	vel[sel_jnt] = limit_vel;
			else if( vel[sel_jnt] < -limit_vel )	vel[sel_jnt] = -limit_vel;
control.move_velo(vel);
run_time = ros::Time::now();
				}
				vel[sel_jnt] = 0.0;
				control.move_velo(vel);
				t_save = run_time = ros::Time::now();
				while( (run_time-t_save).toSec() < 0.75) run_time = ros::Time::now();
				if( fp )
					fclose(fp);
		}
*/
  }
  catch(const std::string &err){
    ROS_ERROR("%s", err.c_str());
  }

  printf("end\n");
  ros::shutdown();
  return 0;
}

bool waitUntilStateChange(cControl &control, int indx, ros::Time &t) {
	sensor_msgs::JointState oldState, newState;
	bool first = true;
	do {
		t = ros::Time::now();
		if( !control.wait_new_joint_state(&newState, 1.0) ) {
			return false;
		}
		if(first) {
			first = false;
			oldState = newState;
		}
	} while( fabs(oldState.velocity[indx]-newState.velocity[indx]) <= 0.001 );
	return true;
}

bool create_trajectory(cControl &control){
  geometry_msgs::Pose start_pose = control.get_current_pose()
    , end_pose = start_pose;

//  start_pose.position.x+= 0.1;

  end_pose.position.x = start_pose.position.x;
  end_pose.position.y = start_pose.position.y;
  end_pose.position.z = start_pose.position.z-0.05;

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

