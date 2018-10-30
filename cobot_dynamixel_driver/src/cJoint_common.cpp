
#include "ros/ros.h"

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <tinyxml.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include "cobot_dynamixel_driver/cJoint.h"


dynamixel::PacketHandler *cJoint::packetHandler = NULL;
dynamixel::PortHandler *cJoint::portHandler = NULL;
dynamixel::GroupSyncWrite *cJoint::group_write_velo = NULL, *cJoint::group_write_velo_acc = NULL, *cJoint::group_write_pos_velo = NULL, *cJoint::group_write_pos_velo_acc = NULL, *cJoint::group_write_acc = NULL, *cJoint::group_write_gain_p = NULL, *cJoint::group_write_goal_torque = NULL;
dynamixel::GroupSyncRead *cJoint::group_read_sync = NULL;
dynamixel::GroupBulkRead *cJoint::group_read_bulk = NULL;
std::vector<cJoint> cJoint::joints;
int cJoint::ADDR[32][2] = {{0}};
//int cJoint::mode = -1;
std::vector<std::string> cJoint::joint_names;
std::string cJoint::setting_file;
bool cJoint::b_set_home = false;

std::string get_attr(TiXmlNode *parent, const char *child_name, const char *attr){
  TiXmlNode *child = parent->FirstChild(child_name);
  if( !child ){
    throw std::string("Node '") + child_name + "' does not exist";
  }
  TiXmlElement* pe = child->ToElement();
  const std::string *val = pe->Attribute(std::string(attr));
  if( !val ){
    throw std::string("Attribute '") + attr + "' in '" + child_name + "' does not exist";
  }
  return *val;
}


double mystof(const std::string &str){
  for(int i=str.size()-1;i>=0;i--){
    char c = str[i];
    if( (c<'0' || c>'9') && c!='-' && c!='.' ){
      throw std::string("stoi : invalid char : ") + str;
    }
  }
  return atof(str.c_str());
}


cJoint::cJoint():id(0),goal_pos(0.0),current(0),velo(0),pos(0),goal_velo(0.0),goal_torque(0.0)
, b_goal_velo(false), b_goal_velo_acc(false), b_goal_pos_velo(false), b_goal_pos_velo_acc(false), b_goal_torque(false){

  motor_model_number = 0;
  cw_angle_limit = 0;
  ccw_angle_limit = 360;
  torque_limit = 900;
  velocity_limit = 2 * M_PI;
  acceleration_limit = 2 * M_PI;
  current_max = 5000;
  rad2val = 0;
  velo2val = 0;
  position_value = 0;
  gear_ratio = 1;
  input_voltage = 0;
  temperature = 0;
  load = 0;
  acc2val = 0;
}

cJoint::cJoint(int _id):cJoint(){ id = _id; }


void cJoint::terminate(){
	if ( group_write_goal_torque ) {
		delete group_write_goal_torque;
		group_write_goal_torque = NULL;	
	}

  if( group_write_gain_p ){
    delete group_write_gain_p;
    group_write_gain_p = NULL;
  }

  if( group_write_velo ){
    delete group_write_velo;
    group_write_velo = NULL;
  }
  if( group_write_velo_acc ){
    delete group_write_velo_acc;
    group_write_velo_acc = NULL;
  }
  if( group_write_acc ){
    delete group_write_acc;
    group_write_acc = NULL;
  }
  if( group_write_pos_velo ){
    delete group_write_pos_velo;
    group_write_pos_velo = NULL;
  }
  if( group_write_pos_velo_acc ){
    delete group_write_pos_velo_acc;
    group_write_pos_velo_acc = NULL;
  }
  if( group_read_sync ){
    delete group_read_sync;
    group_read_sync = NULL;
  }
  if( group_read_bulk ){
    delete group_read_bulk;
    group_read_bulk = NULL;
  }
  if( packetHandler && portHandler ){

    for(int i=0;i<joints.size();i++){
      try{
				ROS_INFO("JOINT[%d] 1: P_SHUTDOWN = %d", i, joints[i].read( P_SHUTDOWN ));
        joints[i].write( P_TORQUE_ENABLE, 0 );
				ROS_INFO("JOINT[%d] 2: P_SHUTDOWN = %d", i, joints[i].read( P_SHUTDOWN ));
      }
      catch(const std::string &err){

        ROS_ERROR("%s", err.c_str());
      }
    }
    portHandler->closePort();
    portHandler = NULL;
    packetHandler = NULL;
  }
  ROS_INFO("joints terminated\n");
}

bool cJoint::is_all_reaching_goal_pos() {
  const int EPS_POS = 3, EPS_VELO = 3;
  for(int i=joints.size()-1;i>=0;i--){
    const cJoint &j = joints[i];
    if( abs(j.pos-j.goal_pos) > EPS_POS/* || abs(j.velo) > EPS_VELO*/ )
      return false;
  }
  return true;
}

std::string cJoint::get_joint_name(int id){
  if( joint_names.empty() )
    return std::string("joint_") + tostr(id);
  else{
    if( id<=0 || id>joint_names.size()){
      mythrow(std::string("Invalid joint number : ") + tostr(id));
    }
    return joint_names[id-1];
  }
}

void cJoint::load_joint_name() {
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  robot_model::RobotModelPtr r = robot_model_loader.getModel();
  if( r==NULL ){
    ROS_WARN("Set param 'load_joint_name' to 'False' if you don't want to use robot model");
    mythrow("No robot model found");
  }
  for(int i=0;i<r->getJointModelCount();i++){
    const robot_state::JointModel *j = r->getJointModel(i);
    const std::string name = j->getName();
    const robot_model::JointModel::JointType type = j->getType();
    if( name.size()==0 || (name[0]!='j' && name[0]!='J') || type!=robot_model::JointModel::REVOLUTE )// name.compare("virtual_joint")==0 )
      continue;
    ROS_INFO(" - %s", name.c_str());
    joint_names.push_back(name);
  }
  if( joint_names.empty() ){
    mythrow("No joint found in robot model");
  }
}

void cJoint::reset_goal() {
  for(int i=joints.size()-1;i>=0;i--) {
    joints[i].b_goal_pos_velo_acc = joints[i].b_goal_pos_velo = joints[i].b_goal_velo = joints[i].b_goal_velo_acc = joints[i].b_goal_torque = false;
  }
}
