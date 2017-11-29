#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library
import math


ADDR = {'MODEL_NUMBER'    : [0, 2]
  ,'ID'                   : [7, 1]
  ,'RETURN_DELAY_TIME'    : [9, 1]
  ,'TORQUE_LIMIT'         : [30, 2]
  ,'CW_ANGLE_LIMIT'       : [36, 4]
  ,'CCW_ANGLE_LIMIT'      : [40, 4]
  ,'TORQUE_ENABLE'        : [562, 1]
  ,'GOAL_POSITION'        : [596, 4]
  ,'GOAL_VELOCITY'        : [600, 4]
  ,'PRESENT_POSITION'     : [611, 4]
  ,'PRESENT_VELOCITY'     : [615, 4]
  ,'PRESENT_CURRENT'      : [621, 2]
  ,'PRESENT_INPUT_VOLTAGE': [623, 2]
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

MODE_POSITION_CONTROL = 0
MODE_VELOCITY_CONTROL = 1
MODE_TORQUE_CONTROL = 2

mode = None


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


class cJoint:
  def __init__(self, _id):
    global group_read, JOINT_INFO
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

    self.model = self.read('MODEL_NUMBER')
    if self.model!=self.info['model_number']:
      raise Exception('[ID %d] model number does not match : %d, %d' % (_id, self.model, self.info['model_number']))
    self.name = 'joint_' + str(_id)
    self.write( 'TORQUE_ENABLE', 0 )
    self.write( 'TORQUE_LIMIT', self.info['torque_max'] )
    self.write( 'CW_ANGLE_LIMIT', self.info['angle_min'] * self.info['angle2val'] )
    self.write( 'CCW_ANGLE_LIMIT', self.info['angle_max'] * self.info['angle2val'] )
    self.write( 'TORQUE_ENABLE', 1 )

    # Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(group_read, _id
      , ADDR['PRESENT_POSITION'][0]
      , ADDR['PRESENT_POSITION'][1] + ADDR['PRESENT_VELOCITY'][1] + ADDR['PRESENT_CURRENT'][1] \
       + ADDR['PRESENT_INPUT_VOLTAGE'][1] + ADDR['PRESENT_TEMPERATURE'][1])).value
    if dxl_addparam_result != 1:
      raise Exception('[ID:%03d] groupBulkRead addparam failed' % (DXL1_ID))

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

  def write(self, param, val):
    global ADDR
    [addr, n_byte] = ADDR[param]
    if n_byte==1:
      cb_write = dynamixel.write1ByteTxRx
    elif n_byte==2:
      cb_write = dynamixel.write2ByteTxRx
    elif n_byte==4:
      cb_write = dynamixel.write4ByteTxRx
    else:
      raise Exception('write() failed : byte = %d' % (n_byte))

    cnt = 5
    while cnt>0:
      #dynamixel.write1ByteTxRx(portHandler, PROTOCOL_VERSION, self.id, addr, val)
      cb_write(portHandler, PROTOCOL_VERSION, self.id, addr, val)
      dxl_comm_result = dynamixel.getLastTxRxResult(portHandler, PROTOCOL_VERSION)
      dxl_error = dynamixel.getLastRxPacketError(portHandler, PROTOCOL_VERSION)
      if dxl_comm_result != COMM_SUCCESS:
        print(('[ID %d] : ' % (self.id)) + dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
      elif dxl_error != 0:
        print(('[ID %d] : ' % (self.id)) + dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
      else:
        return
      cnt-=1
    raise '[ID %d] : write1b failed : addr = %d' % (self.id, addr))

  def read(self, param):
    global ADDR
    [addr, n_byte] = ADDR[param]
    if n_byte==1:
      cb_read = dynamixel.read1ByteTxRx
    elif n_byte==2:
      cb_read = dynamixel.read2ByteTxRx
    elif n_byte==4:
      cb_read = dynamixel.read4ByteTxRx
    else:
      raise Exception('read() failed : byte = %d' % (n_byte))
    val = cb_read(portHandler, PROTOCOL_VERSION, self.id, addr)
    dxl_comm_result = dynamixel.getLastTxRxResult(portHandler, PROTOCOL_VERSION)
    dxl_error = dynamixel.getLastRxPacketError(portHandler, PROTOCOL_VERSION)
    if dxl_comm_result != COMM_SUCCESS:
      print(('[ID %d] : ' % (self.id)) + dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
    elif dxl_error != 0:
      print(('[ID %d] : ' % (self.id)) + dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
    return val


def init():
  global joints, portHandler, DEVICENAME, BAUDRATE, PROTOCOL_VERSION, mode, MODE_VELOCITY_CONTROL
  portHandler = dynamixel.portHandler(DEVICENAME.encode('utf-8'))
  dynamixel.packetHandler()
  group_write_velo = dynamixel.groupSyncWrite(portHandler, PROTOCOL_VERSION
    , ADDR['GOAL_VELOCITY'][0], ADDR['GOAL_VELOCITY'][1])
  group_write_pos_velo = dynamixel.groupSyncWrite(portHandler, PROTOCOL_VERSION
    , ADDR['GOAL_POSITION'][0], ADDR['GOAL_POSITION'][1] + ADDR['GOAL_VELOCITY'][1])
  group_read = dynamixel.groupBulkRead(portHandler, PROTOCOL_VERSION)

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
    j = joints[-1]
    _id = j.read('ID');
    if _id!=i+1:
      raise Exception("id does not match : %d / %d" %(_id, i+1))
  mode = MODE_VELOCITY_CONTROL
  return joints


def sync_velo():
  global mode, MODE_VELOCITY_CONTROL, joints, ADDR, group_write_velo, portHandler, PROTOCOL_VERSION, COMM_SUCCESS
  [addr, n_byte] = ADDR['GOAL_VELOCITY']
  if mode!=MODE_VELOCITY_CONTROL:
    change_mode(MODE_VELOCITY_CONTROL)
  for j in joints:
    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_write_velo, j.id, j.goal_velo, n_byte)).value
    if dxl_addparam_result != 1:
      raise Exception("[ID:%03d] groupSyncWrite addparam failed" % (j.id))

  # Syncwrite goal position
  dynamixel.groupSyncWriteTxPacket(group_write_velo)
  dxl_comm_result = dynamixel.getLastTxRxResult(portHandler, PROTOCOL_VERSION)
  if dxl_comm_result != COMM_SUCCESS:
    raise Exception(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))

  # Clear syncwrite parameter storage
  dynamixel.groupSyncWriteClearParam(group_write_velo)



def sync_pos_velo():
    global mode, MODE_POSITION_CONTROL, joints, ADDR, group_write_pos_velo, portHandler, PROTOCOL_VERSION, COMM_SUCCESS
  if mode!=MODE_POSITION_CONTROL:
    change_mode(MODE_POSITION_CONTROL)
  for j in joints:
    ###########################
    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_write_pos_velo, j.id, j.goal_pos, n_byte)).value
    if dxl_addparam_result != 1:
      raise Exception("[ID:%03d] groupSyncWrite addparam failed" % (j.id))

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



init()
