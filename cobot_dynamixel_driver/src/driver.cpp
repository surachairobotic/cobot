#include "cobot_dynamixel_driver/common.h"

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "cobot_dynamixel_driver/get_motor_number.h"
#include "cobot_dynamixel_driver/set_home.h"
#include "cobot_dynamixel_driver/set_acc.h"
#include "cobot_dynamixel_driver/set_p_gain.h"
#include "cobot_dynamixel_driver/set_i_gain.h"
#include "cobot_dynamixel_driver/set_torque.h"
#include "cobot_dynamixel_driver/up_one.h"
#include "sensor_msgs/JointState.h"
#include "cobot_dynamixel_driver/cJoint.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <unistd.h>
#include <limits.h>

std::string iso_time_str = "";
int num_log = 0;
ros::Publisher pub_return;
ros::Time t, t_callback, t_push;
double timestamp = 0.0;
FILE *_log;
FILE *_cmd;
//std::vector<double> offset{0.0, -0.016406, -0.009949, 0.028798, -0.000698, 0.0};
//std::vector<double> offset_2{-0.04363323,  0.038048,-0.00384,-0.036303,0.011519,0.0};
std::vector<double> offset_2{0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000};
std::vector<double> offset{-0.042760, 0.014835, -0.012218, -0.008727, 0.008727, 0.000000};
bool debug = false;
bool tq_over = false;
bool b_emergency = false;

void control_callback(const sensor_msgs::JointState::ConstPtr& msg);
bool get_motor_number(cobot_dynamixel_driver::get_motor_number::Request  &req, cobot_dynamixel_driver::get_motor_number::Response &res);
bool set_home(cobot_dynamixel_driver::set_home::Request  &req, cobot_dynamixel_driver::set_home::Response &res);
bool set_acc(cobot_dynamixel_driver::set_acc::Request &req, cobot_dynamixel_driver::set_acc::Response &res);
bool set_p_gain(cobot_dynamixel_driver::set_p_gain::Request &req, cobot_dynamixel_driver::set_p_gain::Response &res);
bool set_i_gain(cobot_dynamixel_driver::set_i_gain::Request &req, cobot_dynamixel_driver::set_i_gain::Response &res);
bool set_torque(cobot_dynamixel_driver::set_torque::Request &req, cobot_dynamixel_driver::set_torque::Response &res);
bool get_offset(std::string name, double* offs);

bool up_one(cobot_dynamixel_driver::up_one::Request &req, cobot_dynamixel_driver::up_one::Response &res) {
  res.out = req.in + 1;
  return true;
}

void new_control_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  ROS_INFO("new_control_callback : %s", msg->header.frame_id.c_str());
  return;
}

void torque_callback(const std_msgs::Bool msg) {
  tq_over = msg.data;
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver");
  ros::NodeHandle n;
  boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
  iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

	char username[LOGIN_NAME_MAX];
	getlogin_r(username, LOGIN_NAME_MAX);
	std::string result_dir = "/home/";
	result_dir += username;
	result_dir += "/catkin_ws/src/cobot/cobot_dynamixel_driver/log/";	
  FILE *_fp = fopen( (result_dir + iso_time_str + "_driver_log.txt").c_str() , "w");
  FILE *_fp2 = fopen( (result_dir + iso_time_str + "_driver_cmd.txt").c_str() , "w");
	if(_fp == NULL) {
    ROS_ERROR("Can't open log file.");
		return -1;
	}
	if(_fp2 == NULL) {
    ROS_ERROR("Can't open cmd file.");
		return -1;
	}
  _log = _fp;
  _cmd = _fp2;
  t_callback = ros::Time::now();
  
  pub_return = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/joint_states_return", 1000);
  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/joint_states", 1000);
//  ros::Publisher pub_gain = n.advertise<int>("cobot_dynamixel_driver/p_gain_velo", 1000);
  ros::Subscriber sub = n.subscribe("cobot_dynamixel_driver/goal", 1000, control_callback);
  ros::Subscriber sub_tq = n.subscribe("/cobot/torque_detection", 10, torque_callback);

  ros::Rate loop_rate(50);
/*
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
*/
  ros::ServiceServer service = n.advertiseService("cobot_dynamixel_driver/get_motor_number", get_motor_number);
  ros::ServiceServer service_set_home = n.advertiseService("cobot_dynamixel_driver/set_home", set_home);
  ros::ServiceServer service_set_acc = n.advertiseService("cobot_dynamixel_driver/set_acc", set_acc);
  ros::ServiceServer service_set_p_gain = n.advertiseService("cobot_dynamixel_driver/set_p_gain", set_p_gain);
  ros::ServiceServer service_set_i_gain = n.advertiseService("cobot_dynamixel_driver/set_i_gain", set_i_gain);
  ros::ServiceServer service_set_torque = n.advertiseService("cobot_dynamixel_driver/set_torque", set_torque);
  ros::ServiceServer service_up_one = n.advertiseService("cobot_dynamixel_driver/up_one", up_one);
  
  int fake_joints = 0;
  try{
    ros::NodeHandle nh("~");
    int f = 0;
    nh.getParam("fake_joints", f);
    nh.deleteParam("fake_joints");
    if( f < 0 || f > 6 ){
      ROS_ERROR("invalid fake joints param : %d\n", f);
//      FILE *fp1 = fopen( (result_dir + iso_time_str + "_driver_log.txt").c_str() , "a");
      fprintf(_log, "invalid fake joints param : %d\n", f);
      return 0;
    }
    fake_joints = f;
    ROS_INFO("fake joints : %d\n", fake_joints);
    bool b = false;
    nh.getParam("load_joint_name", b);
    nh.deleteParam("load_joint_name");
    if( b )
      cJoint::load_joint_name();
      
    std::string setting_file;
    nh.getParam("setting_file", setting_file);
    nh.deleteParam("setting_file");
    if( setting_file.size()==0 ){
      ROS_ERROR("No setting_file name found\n");
//      FILE *fp1 = fopen( (result_dir + iso_time_str + "_driver_log.txt").c_str() , "a");
      fprintf(_log, "No setting_file name found\n");
      return 0;
    }
    cJoint::set_setting_file(setting_file);
  }
  catch(const std::string &err){
    printf("error\n");
    ROS_ERROR("%s", err.c_str());
//    FILE *fp1 = fopen( (result_dir + iso_time_str + "_driver_log.txt").c_str() , "a");
    fprintf(_log, "%s\n", err.c_str());
    return 0;
  }

  try{
    std::vector<cJoint> &joints = cJoint::init();
    ROS_INFO("running ...\n");
    int joint_num = fake_joints > joints.size() ? fake_joints : joints.size();
    ros::Time t_prev = ros::Time::now();
    t_push = ros::Time::now();

    sensor_msgs::JointState joint_state;
		double dt;
		int i;
    double voltage[joint_num];
    int temperature[joint_num];
    joint_state.name.resize(joint_num);
    joint_state.position.resize(joint_num);
    joint_state.velocity.resize(joint_num);
    joint_state.effort.resize(joint_num);

    int seq = 0;

/*
    while(ros::ok()){
      t = ros::Time::now();
      dt = (t-t_prev).toSec();
      if(dt > 3.0) break;
    }

    while(ros::ok()){
      t = ros::Time::now();
      dt = (t-t_prev).toSec();
      if(dt > 2.5) {
        for(int i=0; i<6; i++) {
          //cJoint &j = joints[i];
          ROS_INFO("ID[%d] : %s : 1", i, joints[i].get_name().c_str());
          joints[i].write(22, 0);
          ROS_INFO("ID[%d] : %s : 2", i, joints[i].get_name().c_str());
          joints[i].write(6, 1);
          ROS_INFO("ID[%d] : %s : 3", i, joints[i].get_name().c_str());
          joints[i].write(31, 0);
          ROS_INFO("ID[%d] : %s : 4", i, joints[i].get_name().c_str());
          joints[i].write(22, 1);
          joints[i].write(22, 1);
        }
        t_prev = ros::Time::now();
      }
    }
*/
    ros::Time t_disconnect = ros::Time::now();
    bool b_disconnect = false;
    while (ros::ok()){
      t = ros::Time::now();
      seq++;

      if( cJoint::sync_read() ){
//      if( 0 ){

        if(b_emergency) {
          for(int i=joints.size()-1; i>=0; i--)
            joints[i].change_mode_2(1);
          b_emergency = false;
        }

        dt = (t-t_prev).toSec();

        if( dt > 0.05 )
          ROS_WARN("Low update rate : %lf sec", dt);
        else if( dt < 0.005 )
          ROS_WARN("High update rate : %lf sec", dt);
        else
          ;//ROS_INFO("Main rate : %lf sec", dt);

        t_prev = t;
        joint_state.header.stamp = t;
        timestamp = joint_state.header.stamp.toSec();
        joint_state.header.seq = seq;
        for(int i=joints.size()-1;i>=0;i--){
          const cJoint &j = joints[i];
          joint_state.name[i] = j.get_name();
          joint_state.position[i] = j.get_pos() + offset[i] + offset_2[i];
          joint_state.velocity[i] = j.get_velo();
          joint_state.effort[i] = j.get_load();
          voltage[i] = j.get_voltage();
          temperature[i] = j.get_temperature();
//          ROS_INFO("joints[i] = %d", i);
//          j.get_info();

//					if( i == joints.size()-1 ) pub_gain.publish(j.get_p_gain());
					//ROS_INFO("Current J%d : %lf", i, j.get_current());
        }
        for(i=joints.size();i<joint_num;i++){
          joint_state.name[i] = cJoint::get_joint_name(i+1);
          joint_state.position[i] = 0 + offset[i] + offset_2[i];
          joint_state.velocity[i] = 0;
          joint_state.effort[i] = 0;
        }
				if(debug && false){
					ROS_INFO("Call3 : %lf, %lf, %lf, %lf, %lf, %lf", joint_state.position[0]-offset[0]-offset_2[0], joint_state.position[1]-offset[1]-offset_2[1], joint_state.position[2]-offset[2]-offset_2[2], joint_state.position[3]-offset[3]-offset_2[3], joint_state.position[4]-offset[4]-offset_2[4], joint_state.position[5]-offset[5]-offset_2[5]);
					ROS_INFO("Call4 : %lf, %lf, %lf, %lf, %lf, %lf", joint_state.position[0], joint_state.position[1], joint_state.position[2], joint_state.position[3], joint_state.position[4], joint_state.position[5]);
					debug = false;
				}
	      pub.publish(joint_state);
        if((ros::Time::now()-t_push).toSec() > 0.1) {
          fprintf(_log, "%lf", joint_state.header.stamp.toSec());
          fprintf(_log, ",%d", joint_state.header.seq);
          for(int i=0; i<joint_num; i++)
            fprintf(_log, ",%lf", joint_state.position[i]);
          for(int i=0; i<joint_num; i++)
            fprintf(_log, ",%lf", joint_state.velocity[i]);
          for(int i=0; i<joint_num; i++)
            fprintf(_log, ",%lf", joint_state.effort[i]);
          for(int i=0; i<joint_num; i++)
            fprintf(_log, ",%lf", voltage[i]);
          for(int i=0; i<joint_num; i++)
            fprintf(_log, ",%d", temperature[i]);
          fprintf(_log, "\n");
          t_push = ros::Time::now();
        }
        b_disconnect = false;
      }
      else if(!b_disconnect) {
        b_disconnect = true;
        t_disconnect = ros::Time::now();
      }
      else if( (ros::Time::now()-t_disconnect).toSec() > 1.0 ) {
        b_emergency = true;
      }
      ros::spinOnce();
      usleep(10000);
//      loop_rate.sleep();
    }
  }
  catch(int err){
//    FILE *fp1 = fopen( (result_dir + iso_time_str + "_driver_log.txt").c_str() , "a");
    fprintf(_log, "catch(int err) is %d\n", err);
  }
  catch(const std::string &err){
    printf("error\n");
    ROS_ERROR("%s", err.c_str());
//    FILE *fp1 = fopen( (result_dir + iso_time_str + "_driver_log.txt").c_str() , "a");
    fprintf(_log, "%s\n", err.c_str());
  }
  cJoint::terminate();
  return 0;
}

void control_callback(const sensor_msgs::JointState::ConstPtr& data) {
  if(b_emergency) return;
  sensor_msgs::JointState msg;
  msg = *data;
  if(tq_over) {
    msg.position.clear();
    msg.effort.clear();
    for(int i=0; i<6; i++)
      msg.velocity[i] = 0.0000;
  }
  bool b_write = false;
  if((ros::Time::now()-t_callback).toSec() > 0.025) {
    b_write = true;
    t_callback = ros::Time::now();
  }

  if(b_write)
    fprintf(_cmd, "%lf,", timestamp);

//  ROS_INFO("void control_callback frame_id : %s", msg->header.frame_id.c_str());
//	if(msg->position.size() == 6)
//		ROS_INFO("Call1 : %lf, %lf, %lf, %lf, %lf, %lf", msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);
	double pos[6];

  std::string s_info = "-";
  for(int i=0; i<msg.name.size(); i++) {
    if(i>0) s_info += ",";
    else s_info += ':';
    s_info += msg.name[i];
  }
  for(int i=0; i<msg.position.size(); i++) {
    if(i>0) s_info += ",";
    else s_info += ':';
    s_info += std::to_string(msg.position[i]);
  }
  for(int i=0; i<msg.velocity.size(); i++) {
    if(i>0) s_info += ",";
    else s_info += ':';
    s_info += std::to_string(msg.velocity[i]);
  }
  for(int i=0; i<msg.effort.size(); i++) {
    if(i>0) s_info += ",";
    else s_info += ':';
    s_info += std::to_string(msg.effort[i]);
  }
  s_info += ':';
//  ROS_WARN("%s", s_info.c_str());
//  ROS_INFO("%s", s_info.c_str());

  try {
    if( msg.position.size() ) {
      // pos & velo control
      s_info += "pos_";
      if( msg.velocity.size() ) {
        s_info += "vel_";
        if( msg.effort.size() ) {
          s_info += "eff";
          if(b_write)
            fprintf(_cmd, "%s\n", s_info.c_str());
//          ROS_INFO("%s", s_info.c_str());
          if( msg.name.size() != msg.velocity.size() || 
              msg.name.size() != msg.position.size() ||
              msg.name.size() != msg.effort.size())
            throw -1;
          bool b_ok = true;
          for(int i=msg.name.size()-1; i>=0; i--) {
            cJoint *j = cJoint::get_joint(std::string(msg.name[i]));
            if( j ) {
							double off = -99.99;
							if( !get_offset(msg.name[i], &off) )
								break;
							pos[i] = (double)msg.position[i]-off;
              if( !j->set_goal_pos_velo_acc((double)msg.position[i]-off, (double)msg.velocity[i], (double)msg.effort[i]) ) {
                b_ok = false;
                break;
              }
            }
            //else
            //  ROS_ERROR("joint not found : %s", msg->name[i].c_str());
          }
          if( b_ok ) {
            cJoint::sync_pos_velo_acc();
          }
          else
            cJoint::reset_goal();
        }
	      else {
          if(b_write)
            fprintf(_cmd, "cobot_dynamixel_driver/goal|%s\n", s_info.c_str());
//          ROS_INFO("%s", s_info.c_str());
          if( msg.name.size()!=msg.velocity.size() || msg.name.size()!=msg.position.size() ) {
            ROS_ERROR("size not math . . .");
            throw -1;
          }
          bool b_ok = true;
          for(int i=msg.name.size()-1; i>=0; i--) {
            cJoint *j = cJoint::get_joint(std::string(msg.name[i]));
            if( j ) {
							double off = -99.99;
							if( !get_offset(msg.name[i], &off) )
								break;
							pos[i] = (double)msg.position[i]-off;
              if( !j->set_goal_pos_velo((double)msg.position[i]-off, (double)msg.velocity[i]) ) {
                b_ok = false;
                break;
              }
            }
            else
              ROS_ERROR("joint not found : %s", msg.name[i].c_str());
          }
//		      ROS_INFO("b_ok = %d", (int)b_ok);
          if( b_ok ) {
            cJoint::sync_pos_velo();
          }
          else
            cJoint::reset_goal();
	      }
      }
      // pos control
      else {
        s_info += ": else ???";
        if(b_write)
          fprintf(_cmd, "cobot_dynamixel_driver/goal|%s\n", s_info.c_str());
//        ROS_INFO("%s", s_info.c_str());
        if( msg.name.size()!=msg.position.size() ) {
//          ROS_ERROR("%s", s_info);
          throw -1;
        }
      }
    }
    // velo control
    else if( msg.velocity.size() ) {
      s_info += "vel_";
      if( msg.effort.size() ) {
        s_info += "eff";
        if(b_write)
          fprintf(_cmd, "cobot_dynamixel_driver/goal|%s\n", s_info.c_str());
//        ROS_INFO("%s", s_info.c_str());
        if( msg.name.size() != msg.velocity.size() ||
			      msg.name.size() != msg.effort.size())
          throw -1;
        bool b_ok = true;
        for(int i=msg.name.size()-1; i>=0; i--) {
          cJoint *j = cJoint::get_joint(std::string(msg.name[i]));
          if( j ) {
            if( !j->set_goal_velo_acc((double)msg.velocity[i], (double)msg.effort[i])) {
              b_ok = false;
              break;
            }
          }
          else
            ROS_ERROR("joint not found : %s", msg.name[i].c_str());
        }
        if( b_ok ) {
//					ROS_INFO("cJoint::set_goal_velo_acc() : b_ok");
          cJoint::sync_velo_acc();
        }
        else {
//					ROS_INFO("cJoint::set_goal_velo_acc() : b_not_ok");
          cJoint::reset_goal();
	      }
      }
      else {
        if(b_write)
          fprintf(_cmd, "cobot_dynamixel_driver/goal|%s\n", s_info.c_str());
//        ROS_INFO("%s", s_info.c_str());
        if( msg.name.size()!=msg.velocity.size() ) {
          throw -1;
        }
        bool b_ok = true;
        for(int i=msg.name.size()-1;i>=0;i--) {
          cJoint *j = cJoint::get_joint(msg.name[i]);
          if( j ) {
            if( !j->set_goal_velo(msg.velocity[i]) ) {
              b_ok = false;
              break;
            }
          }
          //else
          //  ROS_ERROR("joint not found : %s", msg->name[i].c_str());
        }
        if( b_ok )
          cJoint::sync_velo();
        else
          cJoint::reset_goal();
      }
    }
    else if( msg.effort.size() ) {
      s_info += "eff_";
      if(b_write)
        fprintf(_cmd, "cobot_dynamixel_driver/goal|%s\n", s_info.c_str());
//      ROS_INFO("%s", s_info.c_str());
      if( msg.name.size() != msg.effort.size() )
        throw -1;
      bool b_ok = true;
      for(int i=msg.name.size()-1;i>=0;i--) {
        cJoint *j = cJoint::get_joint(std::string(msg.name[i]));
        if( j ) {
          if( !j->set_goal_torque((double)msg.effort[i]) ) {
            b_ok = false;
            break;
          }
        }
      }
      if( b_ok )
        cJoint::sync_torque();
      else
        cJoint::reset_goal();
    }

  } // try
  catch(int err) {
//    ROS_ERROR("%s", s_info.c_str());
    if(b_write)
      fprintf(_cmd, "cobot_dynamixel_driver/goal|%s\n", s_info.c_str());
    if( err==-1 ) {
      ROS_ERROR("invalid data size : %d, %d, %d, %d / %d\n", (int)msg.name.size()
        , (int)msg.position.size(), (int)msg.velocity.size(), (int)msg.effort.size()
        , (int)cJoint::get_joints().size());
    }
//    FILE *fp = fopen( (result_dir + iso_time_str + "_driver_log.txt").c_str() , "a");
    if(b_write)
      fprintf(_cmd, "%d\n%s\ninvalid data size : %d, %d, %d, %d / %d\n", num_log++, s_info.c_str(), (int)msg.name.size(), (int)msg.position.size(), (int)msg.velocity.size(), (int)msg.effort.size(), (int)cJoint::get_joints().size());
  }
  catch(const std::string &err) {
    ROS_ERROR("%s", err.c_str());
    if(b_write) {
      fprintf(_cmd, "cobot_dynamixel_driver/goal|%s\n", s_info.c_str());
  //    ROS_ERROR("%s", s_info.c_str());

  //    FILE *fp = fopen( (result_dir + iso_time_str + "_driver_log.txt").c_str() , "a");
      fprintf(_cmd, "%d\n%s\n%s\n", num_log++, s_info.c_str(), err.c_str());
    }
  }

  pub_return.publish(msg);
//	debug = true;
//  ROS_INFO("end - control_callback");
//  ros::Time t2 = ros::Time::now();
//  ROS_ERROR("Callback rate : %lf sec", (t2-t1).toSec());        
}

bool get_offset(std::string name, double* offs){
	if(name.size() != 2 || name[0] != 'J')
		return false;
	char txt;
	txt = name[1];
	char* pTxt;
	pTxt = &txt;
	int indx = std::atoi(pTxt)-1;
	if(indx < 0 || indx >= 6)
		return false;
	*offs = offset[indx] + offset_2[indx];
//	ROS_INFO("in:%s, txt:%c, indx:%d, out:%lf", name.c_str(), txt, indx, *offs);
	return true;
}

bool get_motor_number(cobot_dynamixel_driver::get_motor_number::Request  &req, cobot_dynamixel_driver::get_motor_number::Response &res){
  res.num = (short)cJoint::get_joints().size();
  return true;
}

bool set_home(cobot_dynamixel_driver::set_home::Request  &req, cobot_dynamixel_driver::set_home::Response &res){
  cJoint::set_home();
  return true;
}

bool set_acc(cobot_dynamixel_driver::set_acc::Request &req, cobot_dynamixel_driver::set_acc::Response &res){
  for(int i=req.joint_names.size()-1;i>=0;i--){
    cJoint *j = cJoint::get_joint(std::string(req.joint_names[i]));
    if( j ){
      if( !j->set_acc((double)req.accelerations[i]) ){
        break;
      }
    }
    //else
    //  ROS_ERROR("joint not found : %s", msg->name[i].c_str());
  }
  return true;
}

bool set_p_gain(cobot_dynamixel_driver::set_p_gain::Request &req, cobot_dynamixel_driver::set_p_gain::Response &res) {
  cJoint *j = cJoint::get_joint(std::string(req.joint_names));
  if( !j->set_p_gain((int)req.p_gain) )
		return false;
	return true;
}

bool set_i_gain(cobot_dynamixel_driver::set_i_gain::Request &req, cobot_dynamixel_driver::set_i_gain::Response &res) {
  cJoint *j = cJoint::get_joint(std::string(req.joint_names));
  if( !j->set_i_gain((int)req.i_gain) )
		return false;
	return true;
}

bool set_torque(cobot_dynamixel_driver::set_torque::Request &req, cobot_dynamixel_driver::set_torque::Response &res) {
  cJoint *j = cJoint::get_joint(std::string(req.joint_name));
	ROS_INFO("driver.cpp :: set_torque --> %s", req.joint_name.c_str());
  if( !j->set_goal_torque(req.torque) )
		return false;
	return true;
}

