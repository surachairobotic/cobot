
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
#include "cobot_dynamixel_driver/cJoint.h"


dynamixel::PacketHandler *cJoint::packetHandler = NULL;
dynamixel::PortHandler *cJoint::portHandler = NULL;
dynamixel::GroupSyncWrite *cJoint::group_write_velo = NULL, *cJoint::group_write_pos_velo = NULL;
dynamixel::GroupSyncRead *cJoint::group_read_sync = NULL;
dynamixel::GroupBulkRead *cJoint::group_read_bulk = NULL;
std::vector<cJoint> cJoint::joints;
int cJoint::ADDR[32][2] = {{0}};
int cJoint::mode = -1;


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


void cJoint::terminate(){
  if( group_write_velo ){
    delete group_write_velo;
    group_write_velo = NULL;
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
        joints[i].write( P_TORQUE_ENABLE, 0 );
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
