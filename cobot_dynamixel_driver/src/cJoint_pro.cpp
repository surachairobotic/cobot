
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
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
//const double M_PI = 3.14159265359;

#define group_read group_read_sync

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




void cJoint::load_settings(const char *xml_file){
  joints.clear();
  TiXmlDocument doc(xml_file);
  if( doc.LoadFile() ){
    for(TiXmlNode *p=doc.FirstChild();p!=0;p=p->NextSibling()){
      if( p->Type()==TiXmlNode::TINYXML_ELEMENT && strcmp( "joints", p->Value())==0 ){
        for(TiXmlNode *p_joint=p->FirstChild();p_joint!=0;p_joint=p_joint->NextSibling()){
          if( strcmp( "joint", p_joint->Value())!=0 )
            continue;
          cJoint j(joints.size());
          TiXmlNode *p_motor = p_joint->FirstChild("motor");
          if( !p_motor ){
            throw std::string("No motor found in joint node : ") + tostr(j.get_id());
          }
          j.motor_name = get_attr( p_motor, "name", "value");
          j.motor_model_number = mystof(get_attr( p_motor, "model_number", "value"));
          j.position_value = mystof(get_attr( p_motor, "position_value", "value"));
          j.rad2val = j.position_value / M_PI;
          j.gear_ratio = mystof(get_attr( p_motor, "gear_ratio", "value"))
              * mystof(get_attr( p_joint, "gear_ratio", "value"));
          j.velo2val = 60.0 * j.gear_ratio / (2*M_PI); // val = rpm * gear_ratio
          j.cw_angle_limit = mystof(get_attr( p_motor, "cw_angle_limit", "value"))
              * M_PI / 180.0 * j.rad2val; // deg -> rad
          j.ccw_angle_limit = mystof(get_attr( p_motor, "ccw_angle_limit", "value"))
              * M_PI / 180.0 * j.rad2val;
          j.torque_limit = mystof(get_attr( p_motor, "torque_limit", "value"));
          j.velocity_limit = mystof(get_attr( p_motor, "velocity_limit", "value"))
              * M_PI / 180.0 * j.velo2val;
          j.acceleration_limit = mystof(get_attr( p_motor, "acceleration_limit", "value"))
              * M_PI / 180.0 * j.velo2val;
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


void cJoint::set_goal_velo(double rad_per_sec){
  int v = rad_per_sec * velo2val;
  if( v<-this->velocity_limit || v>this->velocity_limit){
    ROS_WARN("[%d] set_goal_pos_velo() : invalid velo : %lf\n", id, rad_per_sec);
    return;
  }
  goal_velo = v;
}

void cJoint::set_goal_pos_velo(double _pos, double _velo){
  int p = _pos * rad2val, v = _velo * velo2val;
  if( p<this->cw_angle_limit || p>this->ccw_angle_limit){
    ROS_WARN("[%d] set_goal_pos_velo() : invalid pos : %lf\n", id, _pos);
    return;
  }
  else if( v<-this->velocity_limit || v>this->velocity_limit){
    ROS_WARN("[%d] set_goal_pos_velo() : invalid velo : %lf\n", id, _velo);
    return;
  }
  goal_pos = p;
  goal_velo = v;
//  printf("goal pos : %.3lf / %d , velo : %.3lf / %d\n", _pos, goal_pos, _velo, goal_velo);
}

double cJoint::get_pos() const {
  if( pos < this->cw_angle_limit || pos > this->ccw_angle_limit ){
    ROS_WARN("[%d] get_pos() : invalid pos val : %d\n", id, pos);
    throw 0;
  }
  return pos / rad2val;
}

double cJoint::get_velo() const {
  if( velo < -this->velocity_limit || velo > this->velocity_limit ){
    ROS_WARN("[%d] get_velo() : invalid velo val : %d\n", id, velo);
    throw 0;
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
  return current / current_max;
}

void cJoint::write(const int param, const int val){
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
    else
      throw std::string("cJoint::write() : wrong n_bytes : param = ") + tostr(param);
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
  throw std::string("cJoint::write() : failed");
}


int cJoint::read(const int param){
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
    packetHandler->printTxRxResult(dxl_comm_result);
    throw 0;
  }
  else if (dxl_error != 0){
    packetHandler->printRxPacketError(dxl_error);
    throw 1;
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
  int model = read(P_MODEL_NUMBER);
  if( model!=motor_model_number ){
    throw std::string("model number does not match : ") + tostr(model)
    + " / " + tostr(motor_model_number);
  }
  name = std::string("joint_") + tostr(id);
  write( P_TORQUE_ENABLE, 0 );
  write( P_TORQUE_LIMIT, torque_limit );
  write( P_CW_ANGLE_LIMIT, cw_angle_limit);
  write( P_CCW_ANGLE_LIMIT, ccw_angle_limit );
  write( P_OPERATING_MODE, MODE_VELOCITY_CONTROL );
  write( P_TORQUE_ENABLE, 1 );

  if( !group_read->addParam(id) ){
    throw std::string("grou_read addparam failed : id = ") + tostr(id);
  }
/*
  if (group_read->addParam(id, P_PRESENT_POSITION, 13) != true){
    throw std::string("grou_read addparam failed : id = ") + tostr(id);
  }*/
}

/// static ///

std::vector<cJoint> &cJoint::init(){

  ADDR[P_MODEL_NUMBER][0] = 0;
  ADDR[P_MODEL_NUMBER][1] = 2;
  ADDR[P_ID][0] = 7;
  ADDR[P_ID][1] = 1;
  ADDR[P_RETURN_DELAY_TIME][0] = 9;
  ADDR[P_RETURN_DELAY_TIME][1] = 1;
  ADDR[P_OPERATING_MODE][0] = 11;
  ADDR[P_OPERATING_MODE][1] = 1;
  ADDR[P_CW_ANGLE_LIMIT][0] = 40;
  ADDR[P_CW_ANGLE_LIMIT][1] = 4;
  ADDR[P_CCW_ANGLE_LIMIT][0] = 36;
  ADDR[P_CCW_ANGLE_LIMIT][1] = 4;
  ADDR[P_TORQUE_ENABLE][0] = 562;
  ADDR[P_TORQUE_ENABLE][1] = 1;
  ADDR[P_GOAL_POSITION][0] = 596;
  ADDR[P_GOAL_POSITION][1] = 4;
  ADDR[P_GOAL_VELOCITY][0] = 600;
  ADDR[P_GOAL_VELOCITY][1] = 4;
  ADDR[P_GOAL_ACCELERATION][0] = 606;
  ADDR[P_GOAL_ACCELERATION][1] = 4;
  ADDR[P_TORQUE_LIMIT][0] = 30;
  ADDR[P_TORQUE_LIMIT][1] = 2;
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

  load_settings("/home/tong/catkin_ws/src/cobot/cobot_dynamixel_driver/src/settings_pro.xml");

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  group_write_velo = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_VELOCITY][0], ADDR[P_GOAL_VELOCITY][1]);
  group_write_pos_velo = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_POSITION][0], ADDR[P_GOAL_POSITION][1] + ADDR[P_GOAL_VELOCITY][1]);
  group_read = new dynamixel::GroupSyncRead(portHandler, packetHandler
      , ADDR[P_PRESENT_POSITION][0], 4+4+2+2+1);
//  group_read = new dynamixel::GroupBulkRead(portHandler, packetHandler);
  // Open port
  if (portHandler->openPort()){
    ROS_INFO("Succeeded to open the port!\n");
  }
  else{
    ROS_ERROR("Failed to open the port!\n");
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
          if( dxl_comm_result==COMM_RX_TIMEOUT ){
            ROS_INFO("ping timeout\n");
            break;
          }
          packetHandler->printTxRxResult(dxl_comm_result);
          ROS_ERROR("Fail to ping : %d , motor_num : %d\n", dxl_comm_result, num_joint);
          throw 0;
        }
      }
      else if( dxl_error != 0 ){
        if( j==0 ){
          packetHandler->printRxPacketError(dxl_error);
          ROS_ERROR("Fail to ping 2 : %d , motor_num : %d\n", dxl_error, num_joint);
          throw 0;
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
  ROS_INFO("motor num : %d\n", num_joint);
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

  for(int i=joints.size()-1;i>=num_joint;i--){
    joints.pop_back();
  }
  for(int i=0;i<joints.size();i++){
    joints[i].setup();
  }
  mode = MODE_VELOCITY_CONTROL;
  return joints;
}

void cJoint::sync_velo(){
  uint8_t velo_lh[4];

  if( mode!=MODE_VELOCITY_CONTROL ){
    change_mode(MODE_VELOCITY_CONTROL);
  }
  for(int i=0;i<joints.size();i++){
    const cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    velo_lh[0] = DXL_LOBYTE(DXL_LOWORD(j.goal_velo));
    velo_lh[1] = DXL_HIBYTE(DXL_LOWORD(j.goal_velo));
    velo_lh[2] = DXL_LOBYTE(DXL_HIWORD(j.goal_velo));
    velo_lh[3] = DXL_HIBYTE(DXL_HIWORD(j.goal_velo));
    dxl_addparam_result = group_write_velo->addParam(j.id, velo_lh);
    if (dxl_addparam_result != true){
      ROS_ERROR("[ID:%03d] group_write_velo addparam failed", j.id);
      throw 0;
    }
  }
  int dxl_comm_result = group_write_velo->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    packetHandler->printTxRxResult(dxl_comm_result);
    throw 0;
  }
  group_write_velo->clearParam();
}


void cJoint::sync_pos_velo(){
  uint8_t lh[8];
  if( mode!=MODE_POSITION_CONTROL ){
    change_mode(MODE_POSITION_CONTROL);
  }
  for(int i=0;i<joints.size();i++){
    const cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    lh[0] = DXL_LOBYTE(DXL_LOWORD(j.goal_pos));
    lh[1] = DXL_HIBYTE(DXL_LOWORD(j.goal_pos));
    lh[2] = DXL_LOBYTE(DXL_HIWORD(j.goal_pos));
    lh[3] = DXL_HIBYTE(DXL_HIWORD(j.goal_pos));
    lh[4] = DXL_LOBYTE(DXL_LOWORD(j.goal_velo));
    lh[5] = DXL_HIBYTE(DXL_LOWORD(j.goal_velo));
    lh[6] = DXL_LOBYTE(DXL_HIWORD(j.goal_velo));
    lh[7] = DXL_HIBYTE(DXL_HIWORD(j.goal_velo));
    dxl_addparam_result = group_write_pos_velo->addParam(j.id, lh);
    if (dxl_addparam_result != true){
      ROS_ERROR("[ID:%03d] group_write_pos_velo addparam failed", j.id);
      throw 0;
    }
  }
  int dxl_comm_result = group_write_pos_velo->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    packetHandler->printTxRxResult(dxl_comm_result);
    throw 0;
  }
  group_write_pos_velo->clearParam();
}

void cJoint::sync_read(){
  int dxl_comm_result = group_read->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    packetHandler->printTxRxResult(dxl_comm_result);
    throw 0;
  }

  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    bool dxl_getdata_result = group_read->isAvailable(j.id, P_PRESENT_POSITION, 4+4+2+2+1);
    if (dxl_getdata_result != true){
      ROS_ERROR("[ID:%03d] group_read getdata failed", joints[i].get_id());
      throw 0;
    }
    j.pos = group_read->getData(j.get_id(), ADDR[P_PRESENT_POSITION][0], ADDR[P_PRESENT_POSITION][1]);
    j.velo = group_read->getData(j.get_id(), ADDR[P_PRESENT_VELOCITY][0], ADDR[P_PRESENT_VELOCITY][1]);
    j.current = group_read->getData(j.get_id(), ADDR[P_PRESENT_CURRENT][0], ADDR[P_PRESENT_CURRENT][1]);
    j.input_voltage = group_read->getData(j.get_id(), ADDR[P_PRESENT_INPUT_VOLTAGE][0], ADDR[P_PRESENT_INPUT_VOLTAGE][1]);
    j.temperature = group_read->getData(j.get_id(), ADDR[P_PRESENT_TEMPERATURE][0], ADDR[P_PRESENT_TEMPERATURE][1]);
  }
}


void cJoint::change_mode(int _mode){
  if( mode==_mode ){
    ROS_WARN("same cotrol mode : %d / %d\n", _mode, mode);
    return;
  }
  for(int i=0;i<joints.size();i++){
    joints[i].write( P_OPERATING_MODE, _mode);
  }
  ROS_INFO("control mode has been changed from %d to %d\n", mode, _mode);
  mode = _mode;
}

