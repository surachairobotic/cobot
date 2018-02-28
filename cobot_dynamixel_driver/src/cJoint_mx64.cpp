
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

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
//const double M_PI = 3.14159265359;

#define group_read group_read_bulk
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

inline int f_velo2val(double velo){
  const double RAD2VAL = (1023.0 * 60) / (2 * M_PI * 117.07);
  int v = velo * RAD2VAL;
  if( v > 1023 || v < -1023 ){
    //ROS_ERROR("velo2val() : invalid goal velo : %.3lf / val = %d\n", velo, v);
    //throw 0;
    mythrow(std::string("velo2val() : invalid goal velo : ") + tostr(velo) + " / val : " + tostr(v));
  }
  return v >= 0 ? v : 1024 - v;
}

inline int f_acc2val(double acc){
  const double ACC2VAL = 180.0 / (8.583 * M_PI);
  int val = acc * ACC2VAL;
  if( val<0 || val>254 ){
    mythrow(std::string("acc2val() : invalid acc val : ") + tostr(acc) + " / val = " + tostr(val));
  }
  return val;
}

inline double f_val2velo(int val){
  const double VAL2RAD = (2 * M_PI * 117.07) / (1023.0 * 60);

  if( val<0 || val>=2047){
    mythrow(std::string("val2velo() : invalid velo val : ") + tostr(val));
  }
  return (val<1024 ? val : 1024-val) * VAL2RAD;
}

/*
inline int pos2val(double pos){
  const double RAD2VAL = 4095.0 / (2*M_PI); // M_PI / (180.0 * 0.088);
  int p = pos * RAD2VAL;
  if( p > 4095 || p < 0 ){
    ROS_WARN("pos2val() : invalid goal pos : %.3lf / val = %d\n", pos, p);
    throw 0;
  }
  return p;
}
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
            mythrow(std::string("No motor found in joint node : ") + tostr(j.get_id()));
          }
          j.motor_name = get_attr( p_motor, "name", "value");
          j.motor_model_number = mystof(get_attr( p_motor, "model_number", "value"));
          j.position_value = mystof(get_attr( p_motor, "position_value", "value"));
          j.rad2val = j.position_value / (2*M_PI);
          j.gear_ratio = mystof(get_attr( p_motor, "gear_ratio", "value"))
              * mystof(get_attr( p_joint, "gear_ratio", "value"));
//          j.velo2val = 60.0 * j.gear_ratio / (2*M_PI); // val = rpm * gear_ratio
          j.cw_angle_limit = mystof(get_attr( p_motor, "cw_angle_limit", "value"))
              * M_PI / 180.0 * j.rad2val; // deg -> rad
          j.ccw_angle_limit = mystof(get_attr( p_motor, "ccw_angle_limit", "value"))
              * M_PI / 180.0 * j.rad2val;
          j.torque_limit = mystof(get_attr( p_motor, "torque_limit", "value"));
          j.velocity_limit = f_velo2val( mystof(get_attr( p_motor, "velocity_limit", "value"))
              * M_PI / 180.0 );
          j.acceleration_limit = f_acc2val( mystof(get_attr( p_motor, "acceleration_limit", "value"))
              * M_PI / 180.0 );
          //j.current_max = mystof(get_attr( p_motor, "current_max", "value"));
          if( j.cw_angle_limit >= j.ccw_angle_limit ){
            mythrow(std::string(" joint ") + tostr(joints.size()) + " : cw is smaller than ccw : "
                + tostr(j.cw_angle_limit) + " , " + tostr(j.ccw_angle_limit));
          }

          if (group_read_bulk->addParam(j.id, ADDR[P_PRESENT_POSITION][0], group_read_size) != true){
            mythrow(std::string("grou_read_bulk addparam failed : id = ") + tostr(j.id));
          }
          joints.push_back(j);
        }
      }
    }
  }
  else{
    mythrow(std::string("cannot load xml file : ") + xml_file);
  }
}


bool cJoint::set_goal_velo(double rad_per_sec){
  goal_velo = f_velo2val(rad_per_sec);
  return true;
}

bool cJoint::set_goal_pos_velo(double _pos, double _velo){
  const double PI2 = 2*M_PI;
  while(_pos<0)
    _pos+= PI2;
  while(_pos>PI2)
    _pos-= PI2;
  goal_pos = _pos * rad2val;
  goal_velo = f_velo2val(_velo);
//  printf("goal pos : %.3lf / %d , velo : %.3lf / %d\n", _pos, goal_pos, _velo, goal_velo);
  return true;
}


double cJoint::get_pos() const {
  if( pos < 0 || pos > 4096 ){
    mythrow(std::string("invalid raw pos : id = ") + tostr(id) + " , pos = " + tostr(pos));
  }
  const double VAL2RAD = (2*M_PI) / 4095.0;
  return pos * VAL2RAD;
}

double cJoint::get_velo() const {
  int v = velo >= 1024 ? 1024 - velo : velo;
  if( v > 1023 || v < -1023 ){
    mythrow(std::string("invalid velo val : id = ") + tostr(id) + " , val = " + tostr(velo));
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


void cJoint::write(const int param, const int val){
  int addr = ADDR[param][0];
  int n_bytes = ADDR[param][1];
  int cnt = 1;

  do{
    uint8_t dxl_error = 0;
    int dxl_comm_result;
    if( n_bytes==1 )
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (uint8_t)val, &dxl_error);
    else if(n_bytes==2 )
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (uint16_t)val, &dxl_error);
    else
      mythrow(std::string("cJoint::write() : wrong n_bytes : param = ") + tostr(param));
    if (dxl_comm_result != COMM_SUCCESS){
      ROS_ERROR("write() comm : id = %d, param = %d, addr = %d, n = %d, val = %d", id, param, addr, n_bytes, val);
      ROS_ERROR("%s" , packetHandler->getTxRxResult(dxl_comm_result));
      //mythrow(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
      ROS_ERROR("write() error : id = %d, param = %d, addr = %d, n = %d, val = %d", id, param, addr, n_bytes, val);
      ROS_ERROR("%s" , packetHandler->getRxPacketError(dxl_error));
    //  mythrow(packetHandler->getRxPacketError(dxl_error));
    }
    else{
      return;
    }
  }
  while(--cnt>0);
  mythrow("write() failed");
}


int cJoint::read(const int param){
  int addr = ADDR[param][0];
  int n_bytes = ADDR[param][1];
  uint8_t dxl_error = 0;
  int dxl_comm_result, val;
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
  else{
    mythrow(std::string("cJoint::read() : invalid n_bytes : param = ") + tostr(param));
  }
  if (dxl_comm_result != COMM_SUCCESS){
    ROS_ERROR("read() comm : id = %d, param = %d, addr = %d, n = %d", id, param, addr, n_bytes);
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0){
    ROS_ERROR("read() error : id = %d, param = %d, addr = %d, n = %d", id, param, addr, n_bytes);
    mythrow(packetHandler->getRxPacketError(dxl_error));
  }
  return val;
}

void cJoint::print_data() const {
  ROS_INFO("[%d] : pos = %d, velo = %d, load = %d\n", id, pos, velo, load);
}


void cJoint::setup(){
  int model = read(P_MODEL_NUMBER);
  if( model!=motor_model_number ){
    mythrow(std::string("model number does not match : ") + tostr(model)
    + " / " + tostr(motor_model_number));
  }
  int id = read(P_ID);
  if( id!=this->id ){
    mythrow(std::string("id does not match : ") + tostr(id) + " / " + tostr(this->id));
  }
  name = get_joint_name(id);
//  name = std::string("joint_") + tostr(id);
  write( P_TORQUE_ENABLE, 0 );
  write( P_TORQUE_CONTROL_MODE, 0 );
  write( P_TORQUE_LIMIT, torque_limit );
  write( P_MAX_TORQUE, torque_limit );
  write( P_CW_ANGLE_LIMIT, 0);//cw_angle_limit);
  write( P_CCW_ANGLE_LIMIT, 0);//ccw_angle_limit );
  write( P_TORQUE_ENABLE, 1 );

  int val;
  ROS_INFO("%s", name.c_str());
  val = read(P_TORQUE_CONTROL_MODE);
  ROS_INFO(" TORQUE_CONTROL_MODE : %d", val);
  val = read(P_TORQUE_LIMIT);
  ROS_INFO(" TORQUE_LIMIT : %d", val);
  val = read(P_MAX_TORQUE);
  ROS_INFO(" MAX_TORQUE : %d", val);
  val = read(P_CW_ANGLE_LIMIT);
  ROS_INFO(" CW_ANGLE_LIMIT : %d", val);
  val = read(P_CCW_ANGLE_LIMIT);
  ROS_INFO(" CCW_ANGLE_LIMIT : %d\n", val);
}



/// static ///

std::vector<cJoint> &cJoint::init(){

  ADDR[P_MODEL_NUMBER][0] = 0;
  ADDR[P_MODEL_NUMBER][1] = 2;
  ADDR[P_ID][0] = 3;
  ADDR[P_ID][1] = 1;
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
  ADDR[P_GOAL_TORQUE][0] = 71;
  ADDR[P_GOAL_TORQUE][1] = 2;
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
  ADDR[P_TORQUE_CONTROL_MODE][0] = 70;
  ADDR[P_TORQUE_CONTROL_MODE][1] = 1;
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  group_write_velo = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_VELOCITY][0], ADDR[P_GOAL_VELOCITY][1]);
  group_write_pos_velo = new dynamixel::GroupSyncWrite(portHandler, packetHandler
      , ADDR[P_GOAL_POSITION][0], ADDR[P_GOAL_VELOCITY][1] + ADDR[P_GOAL_POSITION][1]);
  group_read_bulk = new dynamixel::GroupBulkRead(portHandler, packetHandler);
  group_read_size = ADDR[P_PRESENT_TEMPERATURE][0] + ADDR[P_PRESENT_TEMPERATURE][1]
    - ADDR[P_PRESENT_POSITION][0];

  load_settings(setting_file);

  // Open port
  if (portHandler->openPort()){
    ROS_INFO("Succeeded to open the port!\n");
  }
  else{
    mythrow("Failed to open the port!\n");
  }
  ROS_INFO("3\n");

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    ROS_INFO("Succeeded to change the baudrate!\n");
  }
  else{
    mythrow("Failed to change the baudrate!\n");
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
          j = -1;
          if( dxl_comm_result==COMM_RX_TIMEOUT ){
            ROS_INFO("ping timeout\n");
            break;
          }
          ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
          mythrow(std::string("Fail to ping 1 : ") + tostr(num_joint));
        }
      }
      else if( dxl_error != 0 ){
        if( j==0 ){
          ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
          mythrow(std::string("Fail to ping 2 : ") + tostr(num_joint));
        }
      }
      else
        break;
    }
    if( j<0 )
      break;
  }
  if( num_joint==0 ){
    mythrow("no motor found\n");
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
  /*
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
  uint8_t velo_lh[2];

  if( mode!=MODE_VELOCITY_CONTROL ){
    change_mode(MODE_VELOCITY_CONTROL);
  }
  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    if( !j.b_goal_velo )
      continue;
    j.b_goal_velo = false;
    velo_lh[0] = DXL_LOBYTE(j.goal_velo);
    velo_lh[1] = DXL_HIBYTE(j.goal_velo);
    dxl_addparam_result = group_write_velo->addParam(j.id, velo_lh);
    if (dxl_addparam_result != true){
      mythrow(std::string("group_write_velo addparam failed : id = ") + tostr(j.id));
    }
//    printf("velo : %d\n", j.goal_velo);
  }
  int dxl_comm_result = group_write_velo->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
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
    cJoint &j = joints[i];
    bool dxl_addparam_result = false;
    if( !j.b_goal_pos_velo )
      continue;
    j.b_goal_pos_velo = false;
    lh[0] = DXL_LOBYTE(j.goal_pos);
    lh[1] = DXL_HIBYTE(j.goal_pos);
    lh[2] = DXL_LOBYTE(j.goal_velo);
    lh[3] = DXL_HIBYTE(j.goal_velo);
    dxl_addparam_result = group_write_pos_velo->addParam(j.id, lh);
    if (dxl_addparam_result != true){
      mythrow(std::string("[ID:%03d] group_write_pos_velo addparam failed") + tostr(j.id));
    }
  }
  int dxl_comm_result = group_write_pos_velo->txPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    mythrow(packetHandler->getTxRxResult(dxl_comm_result));
  }
  group_write_pos_velo->clearParam();
}

bool cJoint::sync_read(){
  int dxl_comm_result = group_read->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS){
    ROS_WARN("%s", packetHandler->getTxRxResult(dxl_comm_result));
    return false;
    //mythrow(packetHandler->getTxRxResult(dxl_comm_result));
  }

  for(int i=0;i<joints.size();i++){
    cJoint &j = joints[i];
    bool dxl_getdata_result = group_read->isAvailable(j.get_id()
        , ADDR[P_PRESENT_POSITION][0], group_read_size);
    if (dxl_getdata_result != true){
      mythrow(std::string("group_read getdata failed : id = ") + tostr(joints[i].get_id()));
    }
    j.pos = group_read->getData(j.get_id(), ADDR[P_PRESENT_POSITION][0], ADDR[P_PRESENT_POSITION][1]);
    j.velo = group_read->getData(j.get_id(), ADDR[P_PRESENT_VELOCITY][0], ADDR[P_PRESENT_VELOCITY][1]);
    j.load = group_read->getData(j.get_id(), ADDR[P_PRESENT_LOAD][0], ADDR[P_PRESENT_LOAD][1]);
    j.input_voltage = group_read->getData(j.get_id(), ADDR[P_PRESENT_INPUT_VOLTAGE][0], ADDR[P_PRESENT_INPUT_VOLTAGE][1]);
    j.temperature = group_read->getData(j.get_id(), ADDR[P_PRESENT_TEMPERATURE][0], ADDR[P_PRESENT_TEMPERATURE][1]);
  }
  return true;
}


void cJoint::change_mode(int _mode){
  if( mode==_mode ){
    ROS_WARN("same cotrol mode : %d / %d\n", _mode, mode);
    return;
  }
  for(int i=0;i<joints.size();i++){
    switch(_mode){
    case MODE_POSITION_CONTROL:
//      joints[i].write( P_TORQUE_ENABLE, 0 );
      joints[i].write( P_CW_ANGLE_LIMIT, 0);
      joints[i].write( P_CCW_ANGLE_LIMIT, 4095);
      usleep(200000); // <- dont know why but need to prevent 'there is no status packet' error
      joints[i].write( P_TORQUE_CONTROL_MODE, 0);
//      joints[i].write( P_TORQUE_ENABLE, 1 );
      break;
    case MODE_VELOCITY_CONTROL:
      joints[i].write( P_CW_ANGLE_LIMIT, 0);
      joints[i].write( P_CCW_ANGLE_LIMIT, 0);
      joints[i].write( P_GOAL_VELOCITY, 0);
      joints[i].write( P_TORQUE_CONTROL_MODE, 0);
      break;
    case MODE_TORQUE_CONTROL:
      joints[i].write( P_TORQUE_CONTROL_MODE, 1);
      break;
    default:
      ROS_ERROR("cJoint::change_mode() : invalid mode %d\n", mode);
    }
  }
  ROS_INFO("control mode changed from %d to %d\n", mode, _mode);
  mode = _mode;
}

