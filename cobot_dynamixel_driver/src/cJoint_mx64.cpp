
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
#include "cobot_dynamixel_driver/cJoint.h"

enum eADDR{
  P_RETURN_DELAY_TIME,
  P_CW_ANGLE_LIMIT,
  P_CCW_ANGLE_LIMIT,
  P_MAX_TORQUE,
  P_TORQUE_ENABLE,
  P_GOAL_POSITION,
  P_GOAL_VELOCITY,
  P_GOAL_ACCELERATION,
  P_TORQUE_LIMIT,
  P_PRESENT_POSITION,
  P_PRESENT_VELOCITY,
  P_PRESENT_LOAD,
  P_PRESENT_INPUT_VOLTAGE,
  P_PRESENT_TEMPERATURE
};

int ADDR[32][2] = {{0}};

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
//const double M_PI = 3.14159265359;

dynamixel::PacketHandler *cJoint::packetHandler = NULL;
dynamixel::PortHandler *cJoint::portHandler = NULL;
dynamixel::GroupSyncWrite *cJoint::group_write_velo = NULL, *cJoint::group_write_pos_velo = NULL;
dynamixel::GroupSyncRead *cJoint::group_read = NULL;
std::vector<cJoint> cJoint::joints;
int cJoint::mode = -1;


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
}

cJoint::cJoint(int _id):cJoint(){ id = _id; }

inline int velo2val(double velo){
  const double RAD2VAL = (1023.0 * 60) / (2 * M_PI * 117.07);
  int v = velo * RAD2VAL;
  if( v > 1023 || v < -1023 ){
    ROS_WARN("velo2val() : invalid goal velo : %.3lf / val = %d\n", velo, v);
    throw 0;
  }
  return v >= 0 ? v : 1024 - v;
}

inline int pos2val(double pos){
  const double RAD2VAL = 4095.0 / (2*M_PI); // M_PI / (180.0 * 0.088);
  int p = pos * RAD2VAL;
  if( p > 4095 || p < 0 ){
    ROS_WARN("pos2val() : invalid goal pos : %.3lf / val = %d\n", pos, p);
    throw 0;
  }
  return p;
}

void cJoint::set_goal_velo(double rad_per_sec){
  goal_velo = velo2val(rad_per_sec);
}

void cJoint::set_goal_pos_velo(double _pos, double _velo){
  goal_pos = pos2val(_pos);
  goal_velo = velo2val(_velo);
//  printf("goal pos : %.3lf / %d , velo : %.3lf / %d\n", _pos, goal_pos, _velo, goal_velo);
}

double cJoint::get_pos() const {
  if( pos < 0 || pos > 4096 ){
    ROS_WARN("[%d] invalid raw pos : %d\n", id, pos);
    throw 0;
  }
  const double VAL2RAD = (2*M_PI) / 4095.0;
  return pos * VAL2RAD;
}

double cJoint::get_velo() const {
  int v = velo >= 1024 ? 1024 - velo : velo;
  if( v > 1023 || v < -1023 ){
    ROS_WARN("[%d] velo2val() : invalid velo val : %d\n", id, velo);
    throw 0;
  }
  const double VAL2RAD = (2 * M_PI * 117.07) / (1023.0 * 60);
  return v * VAL2RAD;
}

double cJoint::get_load() const {
  const double VAL2LOAD = 1.0 / 1023.0;
  if( load < 0 || load > 2047 ){
    ROS_WARN("[%d] invalid raw load : %d\n", id, load);
  }
  return ( load >= 1024 ? load - 1024 : load ) * VAL2LOAD;
}


void cJoint::write(int addr, int val){
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
  ROS_INFO("[%d] : pos = %d, velo = %d, load = %d\n", id, pos, velo, load);
}



/// static ///

std::vector<cJoint> &cJoint::init(){

  ADDR[P_RETURN_DELAY_TIME][0] = 5;
  ADDR[P_RETURN_DELAY_TIME][1] = 1;
  ADDR[P_CW_ANGLE_LIMIT][0] = 6;
  ADDR[P_CW_ANGLE_LIMIT][1] = 2;
  ADDR[P_CCW_ANGLE_LIMIT][0] = 8;
  ADDR[P_CCW_ANGLE_LIMIT][1] = 2;
  ADDR[P_MAX_TORQUE][0] = 14;
  ADDR[P_MAX_TORQUE][1] = 2;
  ADDR[P_TORQUE_ENABLE][0] = 24;
  ADDR[P_TORQUE_ENABLE][1] = 1;
  ADDR[P_GOAL_POSITION][0] = 30;
  ADDR[P_GOAL_POSITION][1] = 2;
  ADDR[P_GOAL_VELOCITY][0] = 32;
  ADDR[P_GOAL_VELOCITY][1] = 2;
  ADDR[P_GOAL_ACCELERATION][0] = 73;
  ADDR[P_GOAL_ACCELERATION][1] = 1;
  ADDR[P_TORQUE_LIMIT][0] = 34;
  ADDR[P_TORQUE_LIMIT][1] = 2;
  ADDR[P_PRESENT_POSITION][0] = 36;
  ADDR[P_PRESENT_POSITION][1] = 2;
  ADDR[P_PRESENT_VELOCITY][0] = 38;
  ADDR[P_PRESENT_VELOCITY][1] = 2;
  ADDR[P_PRESENT_LOAD][0] = 40;
  ADDR[P_PRESENT_LOAD][1] = 2;
  ADDR[P_PRESENT_INPUT_VOLTAGE][0] = 42;
  ADDR[P_PRESENT_INPUT_VOLTAGE][1] = 1;
  ADDR[P_PRESENT_TEMPERATURE][0] = 43;
  ADDR[P_PRESENT_TEMPERATURE][1] = 1;

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  group_write_velo = new dynamixel::GroupSyncWrite(portHandler, packetHandler, P_GOAL_VELOCITY, 2);
  group_write_pos_velo = new dynamixel::GroupSyncWrite(portHandler, packetHandler, P_GOAL_POSITION, 4);
  group_read = new dynamixel::GroupSyncRead(portHandler, packetHandler);
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
    ROS_ERROR("Press any key to terminate...\n");
    throw 0;
  }

  // ping
  int num_joint = 0;
  uint8_t dxl_error;

  for(;num_joint<250;num_joint++){
    int j = 3;
    for(;j>=0;j--){
      int dxl_comm_result = packetHandler->ping(portHandler, num_joint + 1, &dxl_error);
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
      else
        break;
    }
    if( j<=0 )
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

  joints.clear();
  for(int i=0;i<num_joint;i++){
    int id = i+1;
    joints.push_back(cJoint(id));
    cJoint &j = joints[joints.size()-1];
    std::stringstream ss;
    ss << "joint_" << (i + 1);
    j.name = ss.str();

//    printf("[%d] : torque enable\n", id);
    j.write1b( P_TORQUE_ENABLE, 0 );
//    printf("[%d] : max torque\n", id);
    j.write2b( P_MAX_TORQUE, 912 );
//    printf("[%d] : torque limit\n", id);
    j.write2b( P_TORQUE_LIMIT, 912 );
//    printf("[%d] : torque control\n", id);
    j.write1b( P_TORQUE_CONTROL_MODE, 0 );
//    printf("[%d] : cw angle limit\n", id);
    j.write2b( P_CW_ANGLE_LIMIT, 0);
//    printf("[%d] : ccw angle limit\n", id);
    j.write2b( P_CCW_ANGLE_LIMIT, 0 );
//    printf("[%d] : torque enable\n", id);
    j.write1b( P_TORQUE_ENABLE, 1 );

    if (group_read->addParam(id, P_PRESENT_POSITION, 6) != true){
      ROS_ERROR("[ID:%03d] grou_read addparam failed", id);
      throw 0;
    }
//    printf("[%d] : end\n", id);

  }
  mode = MODE_VELOCITY_CONTROL;
  return joints;
}

void cJoint::sync_velo(){
  uint8_t velo_lh[2];

  if( mode!=MODE_VELOCITY_CONTROL ){
    change_mode(MODE_VELOCITY_CONTROL);
  }
  for(int i=0;i<joints.size();i++){
    const cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    velo_lh[0] = DXL_LOBYTE(j.goal_velo);
    velo_lh[1] = DXL_HIBYTE(j.goal_velo);
    dxl_addparam_result = group_write_velo->addParam(j.id, velo_lh);
    if (dxl_addparam_result != true){
      ROS_ERROR("[ID:%03d] group_write_velo addparam failed", j.id);
      throw 0;
    }
//    printf("velo : %d\n", j.goal_velo);
  }
  int dxl_comm_result = group_write_velo->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    packetHandler->printTxRxResult(dxl_comm_result);
    throw 0;
  }
  group_write_velo->clearParam();
  /*
  for(int i=0;i<joints.size();i++){
    int goal_velo = joints[i].read2b(P_MOVING_SPEED);
    int cw_limit = joints[i].read2b(P_CW_ANGLE_LIMIT);
    int ccw_limit = joints[i].read2b(P_CCW_ANGLE_LIMIT);
    int torque_control = joints[i].read1b(P_TORQUE_CONTROL_MODE);
    int torque_enable = joints[i].read1b(P_TORQUE_ENABLE);

    printf("[%d] sent_goal_velo : %d\ngoal_velo : %d\n  CW : %d\n  CCW : %d\n  TORQUE_CONTROL : %d\n  TORQUE_ENABLE : %d\n"
      , joints[i].get_id(), joints[i].goal_velo, goal_velo, cw_limit, ccw_limit, torque_control, torque_enable );
  }*/
}


void cJoint::sync_pos_velo(){
  uint8_t lh[4];
  if( mode!=MODE_POSITION_CONTROL ){
    change_mode(MODE_POSITION_CONTROL);
  }
  for(int i=0;i<joints.size();i++){
    const cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    lh[0] = DXL_LOBYTE(j.goal_pos);
    lh[1] = DXL_HIBYTE(j.goal_pos);
    lh[2] = DXL_LOBYTE(j.goal_velo);
    lh[3] = DXL_HIBYTE(j.goal_velo);
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
    bool dxl_getdata_result = group_read->isAvailable(j.get_id(), P_PRESENT_POSITION, 6);
    if (dxl_getdata_result != true){
      ROS_ERROR("[ID:%03d] group_read getdata failed", joints[i].get_id());
      throw 0;
    }
    j.pos = group_read->getData(j.get_id(), P_PRESENT_POSITION, 2);
    j.velo = group_read->getData(j.get_id(), P_PRESENT_SPEED, 2);
    j.load = group_read->getData(j.get_id(), P_PRESENT_LOAD, 2);
  }
}


void cJoint::change_mode(int _mode){
  if( mode==_mode ){
    ROS_WARN("same cotrol mode : %d / %d\n", _mode, mode);
    return;
  }
  for(int i=0;i<joints.size();i++){
    switch(_mode){
    case MODE_POSITION_CONTROL:
      joints[i].write2b( P_CW_ANGLE_LIMIT, 0);
      joints[i].write2b( P_CCW_ANGLE_LIMIT, 4095);
      joints[i].write1b( P_TORQUE_CONTROL_MODE, 0);
      break;
    case MODE_VELOCITY_CONTROL:
      joints[i].write2b( P_CW_ANGLE_LIMIT, 0);
      joints[i].write2b( P_CCW_ANGLE_LIMIT, 0);
      joints[i].write2b( P_MOVING_SPEED, 0);
      joints[i].write1b( P_TORQUE_CONTROL_MODE, 0);
      break;
    case MODE_TORQUE_CONTROL:
      joints[i].write1b( P_TORQUE_CONTROL_MODE, 1);
      break;
    default:
      ROS_ERROR("cJoint::change_mode() : invalid mode %d\n", mode);
    }
  }
  ROS_INFO("control mode has been changed from %d to %d\n", mode, _mode);
  mode = _mode;
}

void cJoint::terminate(){
  if( group_write_velo ){
    delete group_write_velo;
    group_write_velo = NULL;
  }
  if( group_read ){
    delete group_read;
    group_read = NULL;
  }
  if( packetHandler && portHandler ){
    for(int i=0;i<joints.size();i++){
      joints[i].write1b( P_TORQUE_ENABLE, 0 );
    }
    portHandler->closePort();
    portHandler = NULL;
    packetHandler = NULL;
  }
  ROS_INFO("joints terminated\n");
}

bool cJoint::is_all_reaching_pos() {
  const int EPS_POS = 3, EPS_VELO = 3;
  for(int i=joints.size()-1;i>=0;i--){
    const cJoint &j = joints[i];
    if( abs(j.pos-j.goal_pos) > EPS_POS/* || abs(j.velo) > EPS_VELO*/ )
      return false;
  }
  return true;
}
