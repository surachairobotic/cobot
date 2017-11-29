#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library
import math


ADDR = {'MODEL_NUMBER'    : [0, 2]
  ,'ID'                   : [7, 1]
  ,'RETURN_DELAY_TIME'    : [9, 1]
  ,'CW_ANGLE_LIMIT'       : [36, 1]
  ,'CCW_ANGLE_LIMIT'      : [40, 1]
  ,'TORQUE_ENABLE'        : [562, 1]
  ,'GOAL_POSITION'        : [596, 1]
  ,'GOAL_VELOCITY'        : [600, 1]
  ,'TORQUE_LIMIT'         : [30, 1]
  ,'PRESENT_POSITION'     : [611, 1]
  ,'PRESENT_VELOCITY'     : [615, 1]
  ,'PRESENT_CURRENT'      : [621, 1]
  ,'PRESENT_INPUT_VOLTAGE': [623, 1]
  ,'PRESENT_TEMPERATURE'  : [625, 1]
}


JOINT_INFO = [
  { 'model_number' : 51200
    , 'model_name' : 'H42-20-S300-R'
    , 'angle2val' : 151875 / math.pi
    , 'angle_max' : math.pi
    , 'angle_min' : -math.pi
    , 'torque_max' : 900
    , 'gear_ratio' : 303.8
    , 'velo_max' : pi
  },
  { 'model_number' : 53768
    , 'model_name' : 'H54-100-S500-R'
    , 'angle2val' : 151875 / math.pi
    , 'angle_max' : math.pi
    , 'angle_min' : -math.pi
    , 'torque_max' : 900
    , 'gear_ratio' : 501.9
    , 'velo_max' : pi
  },
  { 'model_number' : 54024
    , 'model_name' : 'H54-200-S500-R'
    , 'angle2val' : 151875 / math.pi
    , 'angle_max' : math.pi
    , 'angle_min' : -math.pi
    , 'torque_max' : 900
    , 'gear_ratio' : 501.9
    , 'velo_max' : pi
  }
]

class cJoint:
  def __init__(self, _id):
    self.id = _id
    if _id>=len(JOINT_INFO):
      raise Exception('joint id is larger than JOIN_INFO : ' + str(_id))
    self.info = JOINT_INFO[_id-1]
    self.velo2val = 60 * self.info['gear_ratio'] / (2*math.pi)
    self.goal_pos = 0
    self.goal_velo = 0
    self.pos_val = 0
    self.velo_val = 0
    self.current_val = 0
  
  def velo2val(self, velo):
    if fabs(velo) > self.info['velo_max']:
      raise Exception('[ID %d]too large velocity : %f' % (self.id, velo))
    return velo * self.velo2val
    
  def pos2val(self, pos):
    return pos * self.info['angle2val']
    
  def set_goal_velo(self, rad_per_sec):
    self.goal_velo = self.velo2val(rad_per_sec)

  def set_goal_pos_velo(self, pos, velo):
    self.goal_pos = self.pos2val(pos)
    self.goal_velo = self.velo2val(velo)


  def get_pos(self):
    return self.pos_val / self.info['angle2val']
    
  def get_velo():
    return self.velo_val / self.velo2val
    
  def get_current():
    return self.current_val;

  
  def write1b(self, addr, val):
    cnt = 5
    while cnt>0:
      dynamixel.write1ByteTxRx(portHandler, PROTOCOL_VERSION, self.id, addr, val)
      dxl_comm_result = dynamixel.getLastTxRxResult(portHandler, PROTOCOL_VERSION)
      dxl_error = dynamixel.getLastRxPacketError(portHandler, PROTOCOL_VERSION)
      if dxl_comm_result != COMM_SUCCESS:
        print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
        print(('[ID %d] : ' % (self.id)) + dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
      elif dxl_error != 0:
        print(('[ID %d] : ' % (self.id)) + dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
      else:
        return;
      cnt-=1
    raise '[ID %d] : write1b failed : addr = %d' % (self.id, addr))

  
  def write1b(addr, val):
    
  void write2b(int addr, int val);
  int read1b(int addr);
  int read2b(int addr);


# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 57600
DEVICENAME                  = "/dev/ttyUSB0"         # Check which port is being used on your controller
COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed


joints = []
portHandler = None
group_write_velo = None
group_write_pos_velo = None
group_read = None

def init():
  portHandler = dynamixel.portHandler(DEVICENAME.encode('utf-8'))
  dynamixel.packetHandler()
  # Open port
  if dynamixel.openPort(portHandler):
    print("Succeeded to open the port!")
  else:
    raise Exception('cannot open port : ' + str(DEVICENAME))

  # Set port baudrate
  if dynamixel.setBaudRate(portHandler, BAUDRATE):
    print("Succeeded to change the baudrate!")
  else:
    raise Exception('Failed to change the baudrate!')
  
  
  num_joint = 0
  for i in range(250):
    b_ok = False
    for j in range(3):
      dxl_model_number = dynamixel.pingGetModelNum(portHandler, PROTOCOL_VERSION, i+1)
      dxl_comm_result = dynamixel.getLastTxRxResult(portHandler, PROTOCOL_VERSION)
      dxl_error = dynamixel.getLastRxPacketError(portHandler, PROTOCOL_VERSION)
      if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
        b_ok = True
        break
    if not b_ok:
      break
    num_joint+=1

  if num_joint==0:
    raise Exception("no motor found\n")
  
  print('motor num : ' + str(num_joint))

  joints = []
  for i in range(num_joint):
    joints.append(cJoint(i+1))
    
    
    
    joints.push_back(cJoint(i+1));
    cJoint &j = joints[joints.size()-1];
    int id = j.read1b(P_ID);
    if( id!=i+1 ){
      ROS_ERROR("id does not match : %d / %d", id, i+1);
      throw 0;
    }
    int model = j.read1b(P_MODEL_NUMBER);
    
    std::stringstream ss;
    ss << "joint_" << (id);
    j.name = ss.str();
    
    j.write1b( P_TORQUE_ENABLE, 0 );
    
#define MODEL_H42_20_S300_R      51200
#define MODEL_H54_100_S500_R     53768
#define MODEL_H54_200_S500_R     54024

    switch( model ){
      case MODEL_H42_20_S300_R:
        j.write2b( P_TORQUE_LIMIT, TORQUE_LIMIT );
        break;
      case MODEL_H42_20_S300_R:
        j.write2b( P_TORQUE_LIMIT, TORQUE_LIMIT );
        break;
      case MODEL_H42_20_S300_R:
        j.write2b( P_TORQUE_LIMIT, TORQUE_LIMIT );
        break;
      default:
        ROS_ERROR("[ID:%03d] invalid model number : %d", id, model);
        throw 0;
    }
    else if( 
    j.write2b( P_TORQUE_LIMIT, TORQUE_LIMIT );
    j.write2b( P_CW_ANGLE_LIMIT, 0);
    j.write2b( P_CCW_ANGLE_LIMIT, 0 );
    j.write1b( P_TORQUE_ENABLE, 1 );
    
    if (group_read->addParam(id, P_PRESENT_POSITION, 13) != true){
      ROS_ERROR("[ID:%03d] grou_read addparam failed", id);
      throw 0;
    }
//    printf("[%d] : end\n", id);
    
  }
  mode = MODE_VELOCITY_CONTROL;
  return joints;

init()


