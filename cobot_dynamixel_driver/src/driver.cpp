#include "cobot_dynamixel_driver/common.h"

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "test_dynamixel/Goal.h"
#include "cobot_dynamixel_driver/get_motor_number.h"
#include "cobot_dynamixel_driver/set_home.h"
#include "cobot_dynamixel_driver/set_acc.h"
#include "cobot_dynamixel_driver/set_p_gain.h"
#include "cobot_dynamixel_driver/set_i_gain.h"
#include "cobot_dynamixel_driver/set_torque.h"
#include "sensor_msgs/JointState.h"
#include "cobot_dynamixel_driver/cJoint.h"

std::string result_dir = "/home/dell/catkin_ws/src/cobot/cobot_jig_controller/src/";
int num_log = 0;
ros::Publisher pub_return;

void control_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  ROS_INFO("void control_callback frame_id : %s", msg->header.frame_id.c_str());

  std::string s_info = "-";
  for(int i=0; i<msg->name.size(); i++) {
    if(i>0) s_info += ",";
    else s_info += ':';
    s_info += msg->name[i];
  }
  for(int i=0; i<msg->position.size(); i++) {
    if(i>0) s_info += ",";
    else s_info += ':';
    s_info += std::to_string(msg->position[i]);
  }
  for(int i=0; i<msg->velocity.size(); i++) {
    if(i>0) s_info += ",";
    else s_info += ':';
    s_info += std::to_string(msg->velocity[i]);
  }
  for(int i=0; i<msg->effort.size(); i++) {
    if(i>0) s_info += ",";
    else s_info += ':';
    s_info += std::to_string(msg->effort[i]);
  }
  s_info += ':';
//  ROS_WARN("%s", s_info.c_str());
//  ROS_INFO("%s", s_info.c_str());

  try {
    if( msg->position.size() ) {
      // pos & velo control
//      s_info += "pos_";
      if( msg->velocity.size() ) {
//        s_info += "vel_";
        if( msg->effort.size() ) {
//          s_info += "eff";
//          ROS_INFO("%s", s_info.c_str());
          if( msg->name.size() != msg->velocity.size() || 
              msg->name.size() != msg->position.size() ||
              msg->name.size() != msg->effort.size())
            throw -1;
          bool b_ok = true;
          for(int i=msg->name.size()-1; i>=0; i--) {
            cJoint *j = cJoint::get_joint(std::string(msg->name[i]));
            if( j ) {
              if( !j->set_goal_pos_velo_acc((double)msg->position[i], (double)msg->velocity[i], (double)msg->effort[i]) ) {
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
//          ROS_INFO("%s", s_info.c_str());
          if( msg->name.size()!=msg->velocity.size() || msg->name.size()!=msg->position.size() ) {
            ROS_ERROR("size not math . . .");
            throw -1;
          }
          bool b_ok = true;
          for(int i=msg->name.size()-1; i>=0; i--) {
            cJoint *j = cJoint::get_joint(std::string(msg->name[i]));
            if( j ) {
              if( !j->set_goal_pos_velo((double)msg->position[i], (double)msg->velocity[i]) ) {
                b_ok = false;
                break;
              }
            }
            else
              ROS_ERROR("joint not found : %s", msg->name[i].c_str());
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
//        s_info += ": else ???";
//        ROS_INFO("%s", s_info.c_str());
        if( msg->name.size()!=msg->position.size() ) {
//          ROS_ERROR("%s", s_info);
          throw -1;
        }
      }
    }
    // velo control
    else if( msg->velocity.size() ) {
//      s_info += "vel_";
      if( msg->effort.size() ) {
//        s_info += "eff";
//        ROS_INFO("%s", s_info.c_str());
        if( msg->name.size() != msg->velocity.size() ||
			      msg->name.size() != msg->effort.size())
          throw -1;
        bool b_ok = true;
        for(int i=msg->name.size()-1; i>=0; i--) {
          cJoint *j = cJoint::get_joint(std::string(msg->name[i]));
          if( j ) {
            if( !j->set_goal_velo_acc((double)msg->velocity[i], (double)msg->effort[i])) {
              b_ok = false;
              break;
            }
          }
          else
            ROS_ERROR("joint not found : %s", msg->name[i].c_str());
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
//        ROS_INFO("%s", s_info.c_str());
        if( msg->name.size()!=msg->velocity.size() ) {
          throw -1;
        }
        bool b_ok = true;
        for(int i=msg->name.size()-1;i>=0;i--) {
          cJoint *j = cJoint::get_joint(msg->name[i]);
          if( j ) {
            if( !j->set_goal_velo(msg->velocity[i]) ) {
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
    else if( msg->effort.size() ) {
//     s_info += "eff_";
//      ROS_INFO("%s", s_info.c_str());
      if( msg->name.size() != msg->effort.size() )
        throw -1;
      bool b_ok = true;
      for(int i=msg->name.size()-1;i>=0;i--) {
        cJoint *j = cJoint::get_joint(std::string(msg->name[i]));
        if( j ) {
          if( !j->set_goal_torque((double)msg->effort[i]) ) {
            b_ok = false;
            break;
          }
        }
      }
      if( b_ok )
        ;//cJoint::sync_torque();
      else
        cJoint::reset_goal();
    }

  } // try
  catch(int err) {
//    ROS_ERROR("%s", s_info.c_str());
    if( err==-1 ) {
      ROS_ERROR("invalid data size : %d, %d, %d, %d / %d\n", (int)msg->name.size()
        , (int)msg->position.size(), (int)msg->velocity.size(), (int)msg->effort.size()
        , (int)cJoint::get_joints().size());
    }
    FILE *fp = fopen( (result_dir + "driver_log.txt").c_str() , "a");
//    fprintf(fp, "%d\n%s\ninvalid data size : %d, %d, %d, %d / %d\n", num_log++, s_info.c_str(), (int)msg->name.size(), (int)msg->position.size(), (int)msg->velocity.size(), (int)msg->effort.size(), (int)cJoint::get_joints().size());
    if( fp )
      fclose(fp);
  }
  catch(const std::string &err) {
    ROS_ERROR("%s", err.c_str());
//    ROS_ERROR("%s", s_info.c_str());

    FILE *fp = fopen( (result_dir + "driver_log.txt").c_str() , "a");
//    fprintf(fp, "%d\n%s\n%s\n", num_log++, s_info.c_str(), err.c_str());
    if( fp )
      fclose(fp);
  }


  pub_return.publish(msg);
  ROS_INFO("end - control_callback");
//  ros::Time t2 = ros::Time::now();
//  ROS_ERROR("Callback rate : %lf sec", (t2-t1).toSec());        
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
	ROS_INFO("driver.cpp :: set_torque --> %s", req.joint_name);
  if( !j->set_goal_torque(req.torque) )
		return false;
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver");
  ros::NodeHandle n;
  pub_return = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/joint_states_return", 1000);
  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/joint_states", 1000);
//  ros::Publisher pub_gain = n.advertise<int>("cobot_dynamixel_driver/p_gain_velo", 1000);
  ros::Subscriber sub = n.subscribe("cobot_dynamixel_driver/goal", 1000, control_callback);
  ros::ServiceServer service = n.advertiseService("cobot_dynamixel_driver/get_motor_number", get_motor_number);
  ros::ServiceServer service_set_home = n.advertiseService("cobot_dynamixel_driver/set_home", set_home);
  ros::ServiceServer service_set_acc = n.advertiseService("cobot_dynamixel_driver/set_acc", set_acc);
  ros::ServiceServer service_set_p_gain = n.advertiseService("cobot_dynamixel_driver/set_p_gain", set_p_gain);
  ros::ServiceServer service_set_i_gain = n.advertiseService("cobot_dynamixel_driver/set_i_gain", set_i_gain);
  ros::ServiceServer service_set_torque = n.advertiseService("cobot_dynamixel_driver/set_torque", set_torque);
  ros::Rate loop_rate(100);
  int fake_joints = 0;
  try{
    ros::NodeHandle nh("~");
    int f = 0;
    nh.getParam("fake_joints", f);
    nh.deleteParam("fake_joints");
    if( f <0 || f > 6 ){
      ROS_ERROR("invalid fake joints param : %d\n", f);
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
      return 0;
    }
    cJoint::set_setting_file(setting_file);
  }
  catch(const std::string &err){
    printf("error\n");
    ROS_ERROR("%s", err.c_str());
    return 0;
  }

  try{
    std::vector<cJoint> &joints = cJoint::init();
    ROS_INFO("running ...\n");
    int joint_num = fake_joints > joints.size() ? fake_joints : joints.size();
    ros::Time t_prev = ros::Time::now();
    ros::Time t_push = ros::Time::now();

        sensor_msgs::JointState joint_state;
        ros::Time t;
				double dt;
				int i;
        joint_state.name.resize(joint_num);
        joint_state.position.resize(joint_num);
        joint_state.velocity.resize(joint_num);
        joint_state.effort.resize(joint_num);

    while (ros::ok()){
      if( cJoint::sync_read() ){
        t = ros::Time::now();
        dt = (t-t_prev).toSec();
        if( dt > 0.05 )
          ROS_WARN("Low update rate : %lf sec", dt);
        else
          ;//ROS_INFO("Main rate : %lf sec", dt);
        t_prev = t;
        joint_state.header.stamp = t;
//        joint_state.name.resize(joint_num);
//        joint_state.position.resize(joint_num);
//        joint_state.velocity.resize(joint_num);
//        joint_state.effort.resize(joint_num);
        for(int i=joints.size()-1;i>=0;i--){
          const cJoint &j = joints[i];
          joint_state.name[i] = j.get_name();
          joint_state.position[i] = j.get_pos();
          joint_state.velocity[i] = j.get_velo();
          joint_state.effort[i] = j.get_load();
//					if( i == joints.size()-1 ) pub_gain.publish(j.get_p_gain());
					//ROS_INFO("Current J%d : %lf", i, j.get_current());
        }
        for(i=joints.size();i<joint_num;i++){
          joint_state.name[i] = cJoint::get_joint_name(i+1);
          joint_state.position[i] = 0;
          joint_state.velocity[i] = 0;
          joint_state.effort[i] = 0;
        }
        pub.publish(joint_state);
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  catch(int err){
  }
  catch(const std::string &err){
    printf("error\n");
    ROS_ERROR("%s", err.c_str());
  }
  cJoint::terminate();
  return 0;
}
