
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
#include <string>
#include <tinyxml.h>
#include "cobot_dynamixel_driver/cJoint.h"


#define TORQUE_LIMIT    912

#define MODEL_H42_20_S300_R      51200
#define MODEL_H54_100_S500_R     53768
#define MODEL_H54_200_S500_R     54024

#define VELOCITY_LIMIT    3.14


/*

// Control table address
#define P_MODEL_NUMBER        0
#define P_ID                  7
#define P_RETURN_DELAY_TIME   9
#define P_CW_ANGLE_LIMIT      36
#define P_CCW_ANGLE_LIMIT     40
//#define P_MAX_TORQUE          30
#define P_TORQUE_ENABLE       562
#define P_GOAL_POSITION       596
#define P_MOVING_SPEED        600
#define P_TORQUE_LIMIT        30
#define P_PRESENT_POSITION    611
#define P_PRESENT_SPEED       615
#define P_PRESENT_LOAD        621
#define P_PRESENT_INPUT_VOLTAGE   623
#define P_PRESENT_TEMPERATURE     625

#define P_MOVING              610
//#define P_TORQUE_CONTROL_MODE 70
#define P_GOAL_TORQUE         604
#define P_OPERATION_MODE      11
*/


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
//const double M_PI = 3.14159265359;

#define group_read group_read_sync
int group_read_size = 0;

/*
cJoint::cJoint():id(0),goal_pos(0.0),current(0),velo(0),pos(0),goal_velo(0.0),goal_torque(0.0){

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
*/



void cJoint::load_settings(const std::string &xml_file){
  joints.clear();
  TiXmlDocument doc(xml_file.c_str());
  if( doc.LoadFile() ){
    for(TiXmlNode *p=doc.FirstChild();p!=0;p=p->NextSibling()){
      if( p->Type()==TiXmlNode::TINYXML_ELEMENT && strcmp( "joints", p->Value())==0 ){
        for(TiXmlNode *p_joint=p->FirstChild();p_joint!=0;p_joint=p_joint->NextSibling()){
          if( strcmp( "joint", p_joint->Value())!=0 )
            continue;
          cJoint j(joints.size() + 1);
          TiXmlNode *p_motor = p_joint->FirstChild("motor");
          if( !p_motor ){
            throw std::string("No motor found in joint node : ") + tostr(j.get_id());
          }
          j.motor_name = get_attr( p_motor, "name", "value");
          j.motor_model_number = mystof(get_attr( p_motor, "model_number", "value"));
          j.direction = mystof(get_attr( p_joint, "direction", "value"));
          j.position_value = mystof(get_attr( p_motor, "position_value", "value"));
          j.rad2val = j.position_value / M_PI * j.direction;
          j.gear_ratio = mystof(get_attr( p_motor, "gear_ratio", "value"))
              * mystof(get_attr( p_joint, "gear_ratio", "value"));
          j.velo2val = 60.0 * j.gear_ratio / (2*M_PI) * j.direction; // val = rpm * gear_ratio
          j.acc2val = 3600.0 * j.gear_ratio / (2*M_PI*58000.0) * j.direction; // val = rpm^2 * gear_ratio / 58000
          if( j.direction>0.0 ){
            j.cw_angle_limit = mystof(get_attr( p_motor, "cw_angle_limit", "value"))
                * M_PI / 180.0 * j.rad2val; // deg -> rad
            j.ccw_angle_limit = mystof(get_attr( p_motor, "ccw_angle_limit", "value"))
                * M_PI / 180.0 * j.rad2val;
          }
          else{
            j.cw_angle_limit = mystof(get_attr( p_motor, "ccw_angle_limit", "value"))
                * M_PI / 180.0 * j.rad2val; // deg -> rad
            j.ccw_angle_limit = mystof(get_attr( p_motor, "cw_angle_limit", "value"))
                * M_PI / 180.0 * j.rad2val;
          }
          j.torque_limit = mystof(get_attr( p_motor, "torque_limit", "value"));
          j.velocity_limit = mystof(get_attr( p_motor, "velocity_limit", "value"))
              * M_PI / 180.0 * fabs(j.velo2val);
          j.acceleration_limit = mystof(get_attr( p_motor, "acceleration_limit", "value"))
              * M_PI / 180.0 * fabs(j.acc2val);
          j.current_max = mystof(get_attr( p_motor, "current_max", "value"));
          if( j.cw_angle_limit >= j.ccw_angle_limit ){
            throw std::string(" joint ") + tostr(joints.size()) + " : cw is smaller than ccw : "
                + tostr(j.cw_angle_limit) + " , " + tostr(j.ccw_angle_limit);
          }

          joints.push_back(j);
        }
      }
    }
  }
  else{
    throw std::string("cannot load xml file : ") + xml_file;
  }
}

bool cJoint::set_goal_torque(double torque) {
  ROS_INFO("cJoint::set_goal_torque()");
	int tq = (int)(torque*2048.0/current_max);
  b_goal_velo = b_goal_velo_acc = b_goal_pos_velo = b_goal_pos_velo_acc = false;
  if( tq > TORQUE_LIMIT || tq < -TORQUE_LIMIT ) {
		ROS_WARN("cJoint::set_goal_torque : over limit %lf|%d", torque, tq);
		return false;
	}
	goal_torque = tq;
	b_goal_torque = true;
	return true;
}

bool cJoint::set_goal_velo(double rad_per_sec){
//  ROS_INFO("cJoint::set_goal_velo()");
  int v = rad_per_sec * velo2val;
  b_goal_velo_acc = b_goal_pos_velo = b_goal_pos_velo_acc = b_goal_torque = false;
  if( v<-this->velocity_limit || v>this->velocity_limit){
    ROS_WARN("[%d] set_goal_velo() : invalid velo : %lf\n", id, rad_per_sec);
    return false;
  }
  goal_velo = v;
  b_goal_velo = true;
  return true;
}

bool cJoint::set_goal_velo_acc(double _velo, double _acc){
	ROS_INFO("bool cJoint::set_goal_velo_acc(double _velo, double _acc) : %lf, %lf", _velo, _acc);
  int v = _velo * velo2val, a = fabs(_acc * acc2val);
  b_goal_velo = b_goal_pos_velo = b_goal_pos_velo_acc = b_goal_torque = false;
  if( v<-this->velocity_limit || v>this->velocity_limit){
    ROS_WARN("[%d] set_goal_velo_acc() : invalid velo : %lf\n", id, _velo);
    return false;
  }
  else if( a > this->acceleration_limit ) {
		ROS_WARN("a > this->acceleration_limit");
		a = this->acceleration_limit;
	}
	else if( a < -this->acceleration_limit ) {
		ROS_WARN("a < -this->acceleration_limit");
  	a = -this->acceleration_limit;
	}
  goal_velo = v;
	goal_acc  = a;
  b_goal_velo_acc = true;
	ROS_INFO("N_set_goal_velo_acc(goal_velo, goal_acc) : %d, %d", goal_velo, goal_acc);
  return true;
}

bool cJoint::set_goal_pos_velo(double _pos, double _velo){
  ROS_INFO("cJoint::set_goal_pos_velo()");
  int p = _pos * rad2val, v = _velo * velo2val;
  b_goal_velo = b_goal_velo_acc = b_goal_pos_velo_acc = b_goal_torque = false;
  if( p<this->cw_angle_limit || p>this->ccw_angle_limit){
    ROS_WARN("[%d] set_goal_pos_velo() : invalid pos : %lf , raw = %d (min : %.3lf [rad]/ %lf, max : %.3lf [rad]/ %lf)\n"
      , id, _pos, p
      , this->cw_angle_limit / rad2val, this->cw_angle_limit
      , this->ccw_angle_limit / rad2val, this->ccw_angle_limit);
    return false;
  }
  else if( v<-this->velocity_limit || v>this->velocity_limit || v==0){
    ROS_WARN("[%d] set_goal_pos_velo() : invalid velo : %lf\n", id, _velo);
    return false;
  }
//  ROS_WARN("pos  : %.3lf, %d\nvelo : %.3lf, %d", _pos, p, _velo, v);
  goal_pos = p;
  goal_velo = v;
  b_goal_pos_velo = true;
//  printf("goal pos : %.3lf / %d , velo : %.3lf / %d\n", _pos, goal_pos, _velo, goal_velo);
  return true;
}

bool cJoint::set_goal_pos_velo_acc(double _pos, double _velo, double _acc) {
  ROS_INFO("cJoint::set_goal_pos_velo_acc()");
  int p = _pos * rad2val, v = _velo * velo2val, a = fabs(_acc * acc2val);
  b_goal_velo = b_goal_velo_acc = b_goal_pos_velo = b_goal_torque = false;
  if( p<this->cw_angle_limit || p>this->ccw_angle_limit){
    ROS_WARN("[%d] set_goal_pos_velo() : invalid pos : %lf , raw = %d (min : %.3lf [rad]/ %lf, max : %.3lf [rad]/ %lf)\n"
      , id, _pos, p
      , this->cw_angle_limit / rad2val, this->cw_angle_limit
      , this->ccw_angle_limit / rad2val, this->ccw_angle_limit);
    return false;
  }
  else if( v<-this->velocity_limit || v>this->velocity_limit || v==0){
    ROS_WARN("[%d] set_goal_pos_velo() : invalid velo : %lf\n", id, _velo);
    return false;
  }
  else if( a > this->acceleration_limit )
		a = this->acceleration_limit;
	else if( a < -this->acceleration_limit )
  	a = -this->acceleration_limit;
//  ROS_WARN("pos  : %.3lf, %d\nvelo : %.3lf, %d", _pos, p, _velo, v);
  goal_pos = p;
  goal_velo = v;
	goal_acc = a;
  b_goal_pos_velo_acc = true;
//  printf("goal pos : %.3lf / %d , velo : %.3lf / %d\n", _pos, goal_pos, _velo, goal_velo);
  return true;
}

bool cJoint::set_acc(double _acc){
  int a = fabs(_acc * this->acc2val);
  if( a > this->acceleration_limit )
		a = this->acceleration_limit;
	else if( a < -this->acceleration_limit )
  	a = -this->acceleration_limit;
  write( P_GOAL_ACCELERATION, a );
  return true;
}

bool cJoint::set_p_gain(int _p_gain) {
	int a = _p_gain;
  write( P_VELOCITY_P_GAIN, a );
  return true;
}

int cJoint::get_p_gain() {
  return read( P_VELOCITY_P_GAIN );
}

bool cJoint::set_i_gain(int _i_gain) {
  write( P_VELOCITY_I_GAIN, _i_gain );
  return true;
}

int cJoint::get_i_gain() {
  return read( P_VELOCITY_I_GAIN );
}

void cJoint::get_info() const {
/*
  printf("MODEL_NUMBER : %d\n", this->read( P_MODEL_NUMBER ));
  printf("MODEL_INFORMATION : %d\n", this->read( P_MODEL_INFORMATION ));
  printf("FIRMWARE_VERSION : %d\n", this->read( P_FIRMWARE_VERSION ));
  printf("ID : %d\n", this->read( P_ID ));
  printf("BAUD_RATE : %d\n", this->read( P_BAUD_RATE ));
  printf("RETURN_DELAY_TIME : %d\n", this->read( P_RETURN_DELAY_TIME ));
*/
//  printf("OPERATING_MODE : %d\n", read( P_OPERATING_MODE ));
/*
  printf("HOMING_OFFSET : %d\n", this->read( P_HOMING_OFFSET ));
  printf("MOVING_THRESHOLD : %d\n", this->read( P_MOVING_THRESHOLD ));
  printf("TEMPERATURE_LIMIT : %d\n", this->read( P_TEMPERATURE_LIMIT ));
  printf("MAX_VOLTAGE_LIMIT : %d\n", this->read( P_MAX_VOLTAGE_LIMIT ));
  printf("MIN_VOLTAGE_LIMIT : %d\n", this->read( P_MIN_VOLTAGE_LIMIT ));
  printf("ACCELERATION_LIMIT : %d\n", this->read( P_ACCELERATION_LIMIT ));
  printf("TORQUE_LIMIT : %d\n", this->read( P_TORQUE_LIMIT ));
  printf("VELOCITY_LIMIT : %d\n", this->read( P_VELOCITY_LIMIT ));
  printf("MAX_POSITION_LIMIT : %d\n", this->read( P_MAX_POSITION_LIMIT ));
  printf("MIN_POSITION_LIMIT : %d\n", this->read( P_MIN_POSITION_LIMIT ));
*/
//  printf("SHUTDOWN : %d\n", read( P_SHUTDOWN ));
  printf("HARDWARE_ERROR_STATUS : %d\n", read( P_HARDWARE_ERROR_STATUS ));
  
//  return 0.0;
}

double cJoint::get_pos() const {
/*  if( pos < this->cw_angle_limit || pos > this->ccw_angle_limit ){
    ROS_ERROR("[%d] get_pos() : invalid pos val : %d\n", id, pos);
    throw 0;
  }*/
//  ROS_WARN("cpos : %.3lf, %d", pos / rad2val, pos);
  return pos / rad2val;
}

double cJoint::get_velo() const {
  if( velo < -this->velocity_limit || velo > this->velocity_limit ){
    ROS_WARN("[%d] get_velo() : invalid velo val : %d\n", id, velo);
//    throw 0;
  }
  return velo / velo2val;
}

double cJoint::get_current() const {
  if( current > current_max ){
    ROS_WARN("[%d] invalid raw load : %d\n", id, current);
  }
  return current;
}

double cJoint::get_load() const {
  return current * current_max / 2048.0;
}

double cJoint::get_voltage() const {
  return input_voltage * 0.1;
}

int cJoint::get_temperature() const {
  return temperature;
}

void cJoint::write(const int param, const int val){
//  ROS_INFO("cJoint::write()");
  int addr = ADDR[param][0];
  int n_bytes = ADDR[param][1];
  int cnt = 5;
  do{
    uint8_t dxl_error = 0;
    int dxl_comm_result;
    if( n_bytes==1 )
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, val, &dxl_error);
    else if( n_bytes==2 )
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, val, &dxl_error);
    else if( n_bytes==4 )
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, val, &dxl_error);
    else {
      ROS_ERROR("cJoint::write --> P_SHUTDOWN : %d", read( P_SHUTDOWN ));
      ROS_ERROR("cJoint::write --> P_HARDWARE_ERROR_STATUS : %d", read( P_HARDWARE_ERROR_STATUS ));
      throw std::string("cJoint::write() : wrong n_bytes : param = ") + tostr(param);
    }
    if (dxl_comm_result != COMM_SUCCESS){
      ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
      ROS_ERROR("cJoint::write --> P_SHUTDOWN : %d", read( P_SHUTDOWN ));
      ROS_ERROR("cJoint::write --> P_HARDWARE_ERROR_STATUS : %d", read( P_HARDWARE_ERROR_STATUS ));
      //throw packetHandler->getTxRxResult(dxl_comm_result);
      //packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0){
      ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
      ROS_ERROR("cJoint::write --> P_SHUTDOWN : %d", read( P_SHUTDOWN ));
      ROS_ERROR("cJoint::write --> P_HARDWARE_ERROR_STATUS : %d", read( P_HARDWARE_ERROR_STATUS ));
      //throw packetHandler->getRxPacketError(dxl_error);
      //packetHandler->printRxPacketError(dxl_error);
    }
    else{
      return;
    }
  }
  while(cnt-- >= 0);
  ROS_ERROR("cJoint::write() : id = %d, param = %d, addr = %d, n_bytes = %d, val = %d", id, param, addr, n_bytes, val);
  ROS_ERROR("cJoint::write --> P_SHUTDOWN : %d", read( P_SHUTDOWN ));
  ROS_ERROR("cJoint::write --> P_HARDWARE_ERROR_STATUS : %d", read( P_HARDWARE_ERROR_STATUS ));
  throw std::string("cJoint::write() : failed");
}


int cJoint::read(const int param) const {
//  ROS_INFO("cJoint::read()");
  int addr = ADDR[param][0];
  int n_bytes = ADDR[param][1];
  uint8_t dxl_error = 0;
  int val;
  int dxl_comm_result;
  if( n_bytes==1 ){
    uint8_t val2;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, &val2, &dxl_error);
    val = val2;
  }
  else if( n_bytes==2 ){
    uint16_t val2;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, &val2, &dxl_error);
    val = val2;
  }
  else if( n_bytes==4 ){
    uint32_t val2;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, &val2, &dxl_error);
    val = val2;
  }
  else{
    throw std::string("cJoint::read() : invalid n_bytes : param = ") + tostr(param);
  }

  if (dxl_comm_result != COMM_SUCCESS){
    ROS_ERROR("cJoint::read() : id = %d, param = %d, addr = %d, n_bytes = %d", id, param, addr, n_bytes);
    //packetHandler->printTxRxResult(dxl_comm_result);
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0){
    //packetHandler->printRxPacketError(dxl_error);
    //throw 1;
    mythrow(packetHandler->getRxPacketError(dxl_error));
  }
  return val;
}
/*
void cJoint::write1b(int addr, int val){
  int cnt = 5;
  do{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, val, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0){
      packetHandler->printRxPacketError(dxl_error);
    }
    else{
      return;
    }
  }
  while(cnt-->=0);
  throw 0;
}

void cJoint::write2b(int addr, int val){
  int cnt = 5;
  do{
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, val, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0){
      packetHandler->printRxPacketError(dxl_error);
    }
    else{
      return;
    }
  }
  while(cnt-->=0);
  throw 0;
}

int cJoint::read1b(int addr){
  uint8_t dxl_error = 0, val;
  int dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, &val, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS){
    packetHandler->printTxRxResult(dxl_comm_result);
    throw 0;
  }
  else if (dxl_error != 0){
    packetHandler->printRxPacketError(dxl_error);
    throw 1;
  }
  return val;
}

int cJoint::read2b(int addr){
  uint8_t dxl_error = 0;
  uint16_t val;
  int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, &val, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS){
    packetHandler->printTxRxResult(dxl_comm_result);
    throw 0;
  }
  else if (dxl_error != 0){
    packetHandler->printRxPacketError(dxl_error);
    throw 1;
  }
  return val;
}
*/
void cJoint::print_data() const {
  ROS_INFO("[%d] : pos = %d, velo = %d, current = %d\n", id, pos, velo, current);
}

void cJoint::setup(){
  int id = read(P_ID);
  if( id!=this->id ){
    throw std::string("id does not match : ") + tostr(id) + " / " + tostr(this->id);
  }
	ROS_INFO("P_ID is %d", id);
  int model = read(P_MODEL_NUMBER);
  if( model!=motor_model_number ){
    throw std::string("model number does not match : ") + tostr(model)
    + " / " + tostr(motor_model_number);
  }
	ROS_INFO("P_MODEL_NUMBER is %d", model);
  name = get_joint_name(id);
  ROS_INFO("---");
	ROS_INFO("JOINT[%d] : P_SHUTDOWN = %d", id, read( P_SHUTDOWN ));
	ROS_INFO("JOINT[%d] : P_HARDWARE_ERROR_STATUS = %d", id, read( P_HARDWARE_ERROR_STATUS ));
  write( P_TORQUE_ENABLE, 0 );
  write( P_TORQUE_LIMIT, torque_limit );
  write( P_CW_ANGLE_LIMIT, cw_angle_limit);
  write( P_CCW_ANGLE_LIMIT, ccw_angle_limit );
  write( P_OPERATING_MODE, MODE_VELOCITY_CONTROL );
  write( P_TORQUE_ENABLE, 1 );

  if( !group_read->addParam(id) ){
    throw std::string("grou_read addparam failed : id = ") + tostr(id);
  }

  mode = MODE_VELOCITY_CONTROL;

/*
  if (group_read->addParam(id, P_PRESENT_POSITION, 13) != true){
    throw std::string("grou_read addparam failed : id = ") + tostr(id);
  }*/
}

/// static ///

std::vector<cJoint> &cJoint::init(){

  ADDR[P_MODEL_NUMBER][0] = 0;
  ADDR[P_MODEL_NUMBER][1] = 2;
  ADDR[P_MODEL_INFORMATION][0] = 2;
  ADDR[P_MODEL_INFORMATION][1] = 4;
  ADDR[P_FIRMWARE_VERSION][0] = 6;
  ADDR[P_FIRMWARE_VERSION][1] = 1;
  ADDR[P_ID][0] = 7;
  ADDR[P_ID][1] = 1;
  ADDR[P_BAUD_RATE][0] = 8;
  ADDR[P_BAUD_RATE][1] = 1;
  ADDR[P_RETURN_DELAY_TIME][0] = 9;
  ADDR[P_RETURN_DELAY_TIME][1] = 1;
  ADDR[P_OPERATING_MODE][0] = 11;
  ADDR[P_OPERATING_MODE][1] = 1;
  ADDR[P_HOMING_OFFSET][0] = 13;
  ADDR[P_HOMING_OFFSET][1] = 4;
  ADDR[P_MOVING_THRESHOLD][0] = 17;
  ADDR[P_MOVING_THRESHOLD][1] = 4;
  ADDR[P_TEMPERATURE_LIMIT][0] = 21;
  ADDR[P_TEMPERATURE_LIMIT][1] = 1;
  ADDR[P_MAX_VOLTAGE_LIMIT][0] = 22;
  ADDR[P_MAX_VOLTAGE_LIMIT][1] = 2;
  ADDR[P_MIN_VOLTAGE_LIMIT][0] = 24;
  ADDR[P_MIN_VOLTAGE_LIMIT][1] = 2;
  ADDR[P_ACCELERATION_LIMIT][0] = 26;
  ADDR[P_ACCELERATION_LIMIT][1] = 4;
  ADDR[P_TORQUE_LIMIT][0] = 30;
  ADDR[P_TORQUE_LIMIT][1] = 2;
  ADDR[P_VELOCITY_LIMIT][0] = 32;
  ADDR[P_VELOCITY_LIMIT][1] = 4;
  ADDR[P_MAX_POSITION_LIMIT][0] = 36;
  ADDR[P_MAX_POSITION_LIMIT][1] = 4;
  ADDR[P_MIN_POSITION_LIMIT][0] = 40;
  ADDR[P_MIN_POSITION_LIMIT][1] = 4;
	ADDR[P_SHUTDOWN][0] = 48;
	ADDR[P_SHUTDOWN][1] = 1;

  ADDR[P_CCW_ANGLE_LIMIT][0] = 36;
  ADDR[P_CCW_ANGLE_LIMIT][1] = 4;
  ADDR[P_CW_ANGLE_LIMIT][0] = 40;
  ADDR[P_CW_ANGLE_LIMIT][1] = 4;
  ADDR[P_TORQUE_ENABLE][0] = 562;
  ADDR[P_TORQUE_ENABLE][1] = 1;
  ADDR[P_VELOCITY_I_GAIN][0] = 586;
  ADDR[P_VELOCITY_I_GAIN][1] = 2;
  ADDR[P_VELOCITY_P_GAIN][0] = 588;
  ADDR[P_VELOCITY_P_GAIN][1] = 2;
  ADDR[P_GOAL_POSITION][0] = 596;
  ADDR[P_GOAL_POSITION][1] = 4;
  ADDR[P_GOAL_VELOCITY][0] = 600;
  ADDR[P_GOAL_VELOCITY][1] = 4;
	ADDR[P_GOAL_TORQUE][0] = 604;
	ADDR[P_GOAL_TORQUE][1] = 2;
  ADDR[P_GOAL_ACCELERATION][0] = 606;
  ADDR[P_GOAL_ACCELERATION][1] = 4;
  ADDR[P_PRESENT_POSITION][0] = 611;
  ADDR[P_PRESENT_POSITION][1] = 4;
  ADDR[P_PRESENT_VELOCITY][0] = 615;
  ADDR[P_PRESENT_VELOCITY][1] = 4;
  ADDR[P_PRESENT_CURRENT][0] = 621;
  ADDR[P_PRESENT_CURRENT][1] = 2;
  ADDR[P_PRESENT_INPUT_VOLTAGE][0] = 623;
  ADDR[P_PRESENT_INPUT_VOLTAGE][1] = 2;
  ADDR[P_PRESENT_TEMPERATURE][0] = 625;
  ADDR[P_PRESENT_TEMPERATURE][1] = 1;
	ADDR[P_HARDWARE_ERROR_STATUS][0] = 892;
	ADDR[P_HARDWARE_ERROR_STATUS][1] = 4;

  group_read_size = 15;
  
  load_settings(setting_file);

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	group_write_goal_torque = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_TORQUE][0], ADDR[P_GOAL_TORQUE][1]);
  group_write_gain_p = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_VELOCITY_P_GAIN][0], ADDR[P_VELOCITY_P_GAIN][1]);
  group_write_velo = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_VELOCITY][0], ADDR[P_GOAL_VELOCITY][1]);
  group_write_acc = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_ACCELERATION][0], ADDR[P_GOAL_ACCELERATION][1]);
  group_write_velo_acc = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_VELOCITY][0], ADDR[P_GOAL_VELOCITY][1] + ADDR[P_GOAL_ACCELERATION][1] + ADDR[P_GOAL_ACCELERATION][1]);
  group_write_pos_velo = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_POSITION][0], ADDR[P_GOAL_POSITION][1] + ADDR[P_GOAL_VELOCITY][1]);
  group_write_pos_velo_acc = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_POSITION][0], ADDR[P_GOAL_POSITION][1] + ADDR[P_GOAL_VELOCITY][1] + ADDR[P_GOAL_ACCELERATION][1]);
  group_read = new dynamixel::GroupSyncRead(portHandler, packetHandler
      , ADDR[P_PRESENT_POSITION][0], group_read_size);
//  group_read = new dynamixel::GroupBulkRead(portHandler, packetHandler);
  // Open port
  if (portHandler->openPort()){
    ROS_INFO("Succeeded to open the port!\n");
  }
  else{
    ROS_ERROR("Failed to open the port! : %s\n", DEVICENAME);
    throw 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    ROS_INFO("Succeeded to change the baudrate!\n");
  }
  else{
    ROS_ERROR("Failed to change the baudrate!\n");
    throw 0;
  }

  // ping
  int num_joint = 0;
  uint8_t dxl_error;
  uint16_t dxl_model_number;
  for(;num_joint<250;num_joint++){
    int j = 3;
    for(;j>=0;j--){
      int dxl_comm_result = packetHandler->ping(portHandler, num_joint + 1, &dxl_model_number, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS ){
        if( j==0 ){
          j = -1;
          if( dxl_comm_result==COMM_RX_TIMEOUT ){
            ROS_INFO("ping timeout\n");
            break;
          }
          //packetHandler->printTxRxResult(dxl_comm_result);
          ROS_ERROR("Fail to ping : %d , motor_num : %d\n", dxl_comm_result, num_joint);
          mythrow(packetHandler->getTxRxResult(dxl_comm_result));
          //throw 0;
        }
      }
      else if( dxl_error != 0 ){
        if( j==0 ){
          //packetHandler->printRxPacketError(dxl_error);
          ROS_ERROR("Fail to ping 2 : %d , motor_num : %d\n", dxl_error, num_joint);
          mythrow(packetHandler->getRxPacketError(dxl_error));
          //throw 0;
        }
      }
      else{
        break;
      }
    }
    if( j<0 )
      break;
  }
  if( num_joint==0 ){
    ROS_WARN("no motor found\n");
    throw 0;
  }
  ROS_INFO("motor num founded : %d\n", num_joint);
/*  int dxl_comm_result = packetHandler->broadcastPing(portHandler,id_list);
  if (dxl_comm_result != COMM_SUCCESS ){
    packetHandler->printTxRxResult(dxl_comm_result);
    printf("Fail to ping : %d , list : %d\n", dxl_comm_result, id_list.size());
    throw 0;
  }
  if( id_list.empty() ){
    printf("Cannot find any motor\n");
    throw 0;
  }
  for(int i=0;i<id_list.size();i++){
    if( id_list[i]!=i+1 ){
      printf("invalid motor ID [%d] : %d\n", i, id_list[i] );
      throw 0;
    }
  }*/

  for(int i=joints.size()-1;i>=num_joint;i--)
    joints.pop_back();


  for(int i=0;i<joints.size();i++) {
    joints[i].setup();
    joints[i].get_info();
  }

//  mode = MODE_VELOCITY_CONTROL;
  ROS_INFO("motor num used : %d\n", (int)joints.size());

  return joints;
}

bool cJoint::send_p_gain(int _p_gain){
  uint8_t p_lh[4];

    bool dxl_addparam_result = false;
    p_lh[0] = DXL_LOBYTE(_p_gain);
    p_lh[1] = DXL_HIBYTE(_p_gain);
    dxl_addparam_result = group_write_gain_p->addParam(id, p_lh);
    if (dxl_addparam_result != true){
      ROS_ERROR("[ID:%03d] group_write_gain_p addparam failed", id);
		  group_write_gain_p->clearParam();
      throw 0;
    }

  int dxl_comm_result = group_write_gain_p->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    //packetHandler->printTxRxResult(dxl_comm_result);
    //throw 0;
		group_write_gain_p->clearParam();
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
		return false;
  }
  group_write_gain_p->clearParam();
	return true;
}

void cJoint::sync_torque(){
  ROS_INFO("cJoint::sync_torque()");
  uint8_t data_lh[4];

/*  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
		if( j.mode != MODE_TORQUE_CONTROL )
		  j.change_mode(MODE_TORQUE_CONTROL);
	}
*/
  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    if( j.goal_torque > j.torque_limit ){
      ROS_WARN("Goal torque exceeded the limit [%d] : %d", i, j.goal_torque );
      return;
    }
  }
  for(int i=0; i<joints.size(); i++) {
    cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    if( !j.b_goal_torque )
      continue;
		if( j.mode != MODE_TORQUE_CONTROL )
		  j.change_mode(MODE_TORQUE_CONTROL);   
    j.b_goal_torque = false;
    data_lh[0] = DXL_LOBYTE(DXL_LOWORD(j.goal_torque));
    data_lh[1] = DXL_HIBYTE(DXL_LOWORD(j.goal_torque));
    data_lh[2] = DXL_LOBYTE(DXL_HIWORD(j.goal_torque));
    data_lh[3] = DXL_HIBYTE(DXL_HIWORD(j.goal_torque));
    dxl_addparam_result = group_write_goal_torque->addParam(j.id, data_lh);
    if (dxl_addparam_result != true){
      ROS_ERROR("[ID:%03d] group_write_goal_torque addparam failed", j.id);
		  group_write_goal_torque->clearParam();
      throw 0;
    }
  }
  int dxl_comm_result = group_write_goal_torque->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    //packetHandler->printTxRxResult(dxl_comm_result);
    //throw 0;
		group_write_goal_torque->clearParam();
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
  }
  group_write_goal_torque->clearParam();
}


void cJoint::sync_velo(){
//  ROS_INFO("cJoint::sync_velo()");
  uint8_t velo_lh[4];

  for(int i=0;i<joints.size();i++) {
    cJoint &j = joints[i];
		if( j.mode != MODE_VELOCITY_CONTROL )
		  j.change_mode(MODE_VELOCITY_CONTROL);
  }

  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    if( fabs(j.goal_velo / j.velo2val) > VELOCITY_LIMIT ){
      ROS_WARN("Goal velo exceeded the limit [%d] : %.3lf", i, j.goal_velo / j.velo2val );
      return;
    }
  }
  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    if( !j.b_goal_velo )
      continue;
    j.b_goal_velo = false;
    velo_lh[0] = DXL_LOBYTE(DXL_LOWORD(j.goal_velo));
    velo_lh[1] = DXL_HIBYTE(DXL_LOWORD(j.goal_velo));
    velo_lh[2] = DXL_LOBYTE(DXL_HIWORD(j.goal_velo));
    velo_lh[3] = DXL_HIBYTE(DXL_HIWORD(j.goal_velo));
    dxl_addparam_result = group_write_velo->addParam(j.id, velo_lh);
    if (dxl_addparam_result != true){
      ROS_ERROR("[ID:%03d] group_write_velo addparam failed", j.id);
		  group_write_velo->clearParam();
      throw 0;
    }
  }
  int dxl_comm_result = group_write_velo->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    //packetHandler->printTxRxResult(dxl_comm_result);
    //throw 0;
		group_write_velo->clearParam();
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
  }
  group_write_velo->clearParam();
}

void cJoint::sync_velo_acc(){
  ROS_INFO("cJoint::sync_velo_acc()");
  uint8_t velo_lh[4], acc_lh[4];

  for(int i=0;i<joints.size();i++) {
    cJoint &j = joints[i];
		if( j.mode != MODE_VELOCITY_CONTROL )
		  j.change_mode(MODE_VELOCITY_CONTROL);
	}

  for(int i=0; i<joints.size(); i++) {
    cJoint &j = joints[i];
    if( fabs(j.goal_velo / j.velo2val) > VELOCITY_LIMIT ) {
      ROS_WARN("Goal velo exceeded the limit [%d] : %.3lf", i, j.goal_velo / j.velo2val );
      return;
    }
  }

	ros::Time t1, t2, t3;
  for(int i=0; i<joints.size(); i++) {
    cJoint &j = joints[i];
    bool dxl_addparam_result_vel = false, dxl_addparam_result_acc = false;
    if( !j.b_goal_velo_acc )
      continue;
    j.b_goal_velo_acc = false;
    velo_lh[0] = DXL_LOBYTE(DXL_LOWORD(j.goal_velo));
    velo_lh[1] = DXL_HIBYTE(DXL_LOWORD(j.goal_velo));
    velo_lh[2] = DXL_LOBYTE(DXL_HIWORD(j.goal_velo));
    velo_lh[3] = DXL_HIBYTE(DXL_HIWORD(j.goal_velo));

    acc_lh[0] = DXL_LOBYTE(DXL_LOWORD(j.goal_acc));
    acc_lh[1] = DXL_HIBYTE(DXL_LOWORD(j.goal_acc));
    acc_lh[2] = DXL_LOBYTE(DXL_HIWORD(j.goal_acc));
    acc_lh[3] = DXL_HIBYTE(DXL_HIWORD(j.goal_acc));

		t1 = ros::Time::now();
    dxl_addparam_result_acc = group_write_acc->addParam(j.id, acc_lh);
		t2 = ros::Time::now();
    dxl_addparam_result_vel = group_write_velo->addParam(j.id, velo_lh);
		t3 = ros::Time::now();
		ROS_INFO("group_write : %lf, %lf", (t2-t1).toSec(), (t3-t2).toSec());
    if (dxl_addparam_result_acc != true){
      ROS_ERROR("[ID:%03d] group_write_velo_acc addparam failed", j.id);
		  group_write_acc->clearParam();
      throw 0;
    }
    if (dxl_addparam_result_vel != true){
      ROS_ERROR("[ID:%03d] group_write_velo_acc addparam failed", j.id);
		  group_write_velo->clearParam();
      throw 0;
    }
  }
  int dxl_comm_result_acc = group_write_acc->txPacket();
  int dxl_comm_result_vel = group_write_velo->txPacket();
  if (dxl_comm_result_acc != COMM_SUCCESS){
		ROS_ERROR("group_write_velo_acc : dxl_comm_result != COMM_SUCCESS");
    //packetHandler->printTxRxResult(dxl_comm_result_acc);
    //throw 0;
		group_write_acc->clearParam();
    mythrow(packetHandler->getTxRxResult(dxl_comm_result_acc));
  }
  if (dxl_comm_result_vel != COMM_SUCCESS){
		ROS_ERROR("group_write_velo_acc : dxl_comm_result != COMM_SUCCESS");
    //packetHandler->printTxRxResult(dxl_comm_result_vel);
    //throw 0;
		group_write_velo->clearParam();
    mythrow(packetHandler->getTxRxResult(dxl_comm_result_vel));
  }
  group_write_acc->clearParam();
  group_write_velo->clearParam();
}

void cJoint::sync_pos_velo(){
  ROS_INFO("cJoint::sync_pos_velo()");
  uint8_t lh[8];

  for(int i=0;i<joints.size();i++) {
    cJoint &j = joints[i];
		if( j.mode != MODE_POSITION_CONTROL )
		  j.change_mode(MODE_POSITION_CONTROL);
  }
  
  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    if( fabs(j.goal_velo / j.velo2val) > VELOCITY_LIMIT ){
      ROS_WARN("Goal velo exceeded the limit [%d] : %.3lf", i, j.goal_velo / j.velo2val );
      return;
    }
  }
  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    bool dxl_addparam_result = false;
//    ROS_INFO("j[%d].b_goal_pos_velo = %d", i, (int)j.b_goal_pos_velo);
    if( !j.b_goal_pos_velo ) {
      continue;
    }
    j.b_goal_pos_velo = false;
    lh[0] = DXL_LOBYTE(DXL_LOWORD(j.goal_pos));
    lh[1] = DXL_HIBYTE(DXL_LOWORD(j.goal_pos));
    lh[2] = DXL_LOBYTE(DXL_HIWORD(j.goal_pos));
    lh[3] = DXL_HIBYTE(DXL_HIWORD(j.goal_pos));
    lh[4] = DXL_LOBYTE(DXL_LOWORD(j.goal_velo));
    lh[5] = DXL_HIBYTE(DXL_LOWORD(j.goal_velo));
    lh[6] = DXL_LOBYTE(DXL_HIWORD(j.goal_velo));
    lh[7] = DXL_HIBYTE(DXL_HIWORD(j.goal_velo));
    dxl_addparam_result = group_write_pos_velo->addParam(j.get_id(), lh);
    if (dxl_addparam_result != true){
      ROS_ERROR("[ID:%03d] group_write_pos_velo addparam failed", j.get_id());
      throw 0;
    }
  }
  int dxl_comm_result = group_write_pos_velo->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    //packetHandler->printTxRxResult(dxl_comm_result);
    //throw 0;
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
  }
  group_write_pos_velo->clearParam();
}

void cJoint::sync_pos_velo_acc() {
  ROS_INFO("cJoint::sync_pos_velo_acc()");
  uint8_t lh[12];

  for(int i=0;i<joints.size();i++) {
    cJoint &j = joints[i];
		if( j.mode != MODE_POSITION_CONTROL )
		  j.change_mode(MODE_POSITION_CONTROL);
	}
  
  for(int i=0; i<joints.size(); i++){
    cJoint &j = joints[i];
    if( fabs(j.goal_velo / j.velo2val) > VELOCITY_LIMIT ) {
      ROS_WARN("Goal velo exceeded the limit [%d] : %.3lf", i, j.goal_velo / j.velo2val );
      return;
    }
  }

  for(int i=0; i<joints.size(); i++) {
    cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    if( !j.b_goal_pos_velo_acc )
      continue;
    j.b_goal_pos_velo_acc = false;
    lh[0] = DXL_LOBYTE(DXL_LOWORD(j.goal_pos));
    lh[1] = DXL_HIBYTE(DXL_LOWORD(j.goal_pos));
    lh[2] = DXL_LOBYTE(DXL_HIWORD(j.goal_pos));
    lh[3] = DXL_HIBYTE(DXL_HIWORD(j.goal_pos));
    lh[4] = DXL_LOBYTE(DXL_LOWORD(j.goal_velo));
    lh[5] = DXL_HIBYTE(DXL_LOWORD(j.goal_velo));
    lh[6] = DXL_LOBYTE(DXL_HIWORD(j.goal_velo));
    lh[7] = DXL_HIBYTE(DXL_HIWORD(j.goal_velo));
    lh[8] = DXL_LOBYTE(DXL_LOWORD(j.goal_acc));
    lh[9] = DXL_HIBYTE(DXL_LOWORD(j.goal_acc));
    lh[10] = DXL_LOBYTE(DXL_HIWORD(j.goal_acc));
    lh[11] = DXL_HIBYTE(DXL_HIWORD(j.goal_acc));
    dxl_addparam_result = group_write_pos_velo_acc->addParam(j.get_id(), lh);
    if (dxl_addparam_result != true){
      ROS_ERROR("[ID:%03d] group_write_pos_velo_acc addparam failed", j.get_id());
      throw 0;
    }
  }

  int dxl_comm_result = group_write_pos_velo_acc->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    //packetHandler->printTxRxResult(dxl_comm_result);
    //throw 0;
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
  }
  group_write_pos_velo_acc->clearParam();
}

bool cJoint::sync_read(){
//  ROS_INFO("cJoint::sync_read()");
  for(int i=3;i>=0;i--){
    int dxl_comm_result = group_read->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
      if( i==0 ){
        ROS_WARN("%s", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
      }
    }
    else
      break;
  }
  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    bool dxl_getdata_result = group_read->isAvailable(j.get_id(), ADDR[P_PRESENT_POSITION][0], group_read_size);
    if (dxl_getdata_result != true){
      ROS_ERROR("[ID:%03d] group_read getdata failed", j.get_id());
      throw 0;
    }
    j.pos = group_read->getData(j.get_id(), ADDR[P_PRESENT_POSITION][0], ADDR[P_PRESENT_POSITION][1]);
    j.velo = group_read->getData(j.get_id(), ADDR[P_PRESENT_VELOCITY][0], ADDR[P_PRESENT_VELOCITY][1]);
    j.current = (short)group_read->getData(j.get_id(), ADDR[P_PRESENT_CURRENT][0], ADDR[P_PRESENT_CURRENT][1]);
    j.input_voltage = group_read->getData(j.get_id(), ADDR[P_PRESENT_INPUT_VOLTAGE][0], ADDR[P_PRESENT_INPUT_VOLTAGE][1]);
    j.temperature = group_read->getData(j.get_id(), ADDR[P_PRESENT_TEMPERATURE][0], ADDR[P_PRESENT_TEMPERATURE][1]);
		j.goal_torque = (short)group_read->getData(j.get_id(), ADDR[P_GOAL_TORQUE][0], ADDR[P_GOAL_TORQUE][1]);
  }
  
  if( b_set_home ){
    b_set_home = false;
    for(int i=0;i<joints.size();i++){
      cJoint &j = joints[i];
      j.write( P_TORQUE_ENABLE, 0 );
      
      int off = j.read(P_HOMING_OFFSET);
      j.write( P_HOMING_OFFSET, off-joints[i].pos );
      j.write( P_TORQUE_ENABLE, 1 );
    }
  }
  return true;
}


void cJoint::change_mode(int _mode){
  ROS_INFO("cJoint::change_mode()");
  if( mode == _mode ) {
    ROS_WARN("same cotrol mode : %d / %d\n", _mode, mode);
    return;
  }

  write( P_TORQUE_ENABLE, 0 );
  write( P_OPERATING_MODE, _mode);
  if( P_OPERATING_MODE != MODE_TORQUE_CONTROL )
    write( P_GOAL_TORQUE, 0 );
//  write( P_TORQUE_LIMIT, torque_limit );
  write( P_TORQUE_ENABLE, 1 );

  ROS_INFO("control mode has been changed from %d to %d\n", mode, _mode);
  mode = _mode;
}

