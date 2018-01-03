
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
#include "sensor_msgs/JointState.h"
#include "cobot_dynamixel_driver/cJoint.h"

void control_callback(const sensor_msgs::JointState::ConstPtr& msg){
/*  if( msg->header.frame_id!="/base_link" ){
    ROS_WARN("invalid frame_id : %s\n", msg->header.frame_id.c_str());
    return;
  }*/
  
  try{
    if( msg->position.size() ){
      // pos & velo control
      if( msg->velocity.size() ){
        ROS_INFO("cmd pos velo : %s = %lf , %lf", msg->name[0].c_str()
          , msg->position[0], msg->velocity[0]);
        if( msg->name.size()!=msg->velocity.size() || msg->name.size()!=msg->position.size() ){
          throw -1;
        }
        for(int i=msg->name.size()-1;i>=0;i--){
          cJoint *j = cJoint::get_joint(std::string(msg->name[i]));
          if( j )
            j->set_goal_pos_velo((double)msg->position[i], (double)msg->velocity[i]);
          //else
          //  ROS_ERROR("joint not found : %s", msg->name[i].c_str());
        }
        cJoint::sync_pos_velo();
      }
      // pos control
      else{
        if( msg->name.size()!=msg->position.size() ){
          throw -1;
        }
      }
    }
    // velo control
    else if( msg->velocity.size() ){
      ROS_INFO("cmd velo : %s = %lf", msg->name[0].c_str(), msg->velocity[0]);
      if( msg->name.size()!=msg->velocity.size() ){
        throw -1;
      }
      for(int i=msg->name.size()-1;i>=0;i--){
        cJoint *j = cJoint::get_joint(msg->name[i]);
        if( j )
          j->set_goal_velo(msg->velocity[i]);
        //else
        //  ROS_ERROR("joint not found : %s", msg->name[i].c_str());
      }
      cJoint::sync_velo();
    }
  }
  catch(int err){
    if( err==-1 ){
      ROS_ERROR("invalid data size : %d, %d, %d, %d / %d\n", (int)msg->name.size()
        , (int)msg->position.size(), (int)msg->velocity.size(), (int)msg->effort.size()
        , (int)cJoint::get_joints().size());
    }
  }
  catch(const std::string &err){
    ROS_ERROR("%s", err.c_str());
  }
}

bool get_motor_number(cobot_dynamixel_driver::get_motor_number::Request  &req, cobot_dynamixel_driver::get_motor_number::Response &res){
  res.num = (short)cJoint::get_joints().size();
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("cobot_dynamixel_driver/joint_states", 1000);
  ros::Subscriber sub = n.subscribe("cobot_dynamixel_driver/goal", 1000, control_callback);
  ros::ServiceServer service = n.advertiseService("cobot_dynamixel_driver/get_motor_number", get_motor_number);
  ros::Rate loop_rate(10);
  int fake_joints = 0;
  {
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

  try{
    std::vector<cJoint> &joints = cJoint::init();
    ROS_INFO("running ...\n");
    int joint_num = fake_joints > joints.size() ? fake_joints : joints.size();
    while (ros::ok()){
      if( cJoint::sync_read() ){
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(joint_num);
        joint_state.position.resize(joint_num);
        joint_state.velocity.resize(joint_num);
        joint_state.effort.resize(joint_num);
        for(int i=joints.size()-1;i>=0;i--){
          const cJoint &j = joints[i];
          joint_state.name[i] = j.get_name();
          joint_state.position[i] = j.get_pos();
          joint_state.velocity[i] = j.get_velo();
          joint_state.effort[i] = j.get_load();
        }
        for(int i=joints.size();i<joint_num;i++){
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
