#!/usr/bin/env python

import sys
import os
import math
import time
import serial
import rospy
from affbot_kinematics.srv import AffbotSetZero

#!/usr/bin/env python

import math

'''
GEAR_RATIO = [1600.0/(400.0*100)
 ,1600.0/(400.0*88.0)
 ,1600.0/(400.0*37.5)
 ,1600.0/(400.0*2.0)
 ,1600.0/(400.0*2.0)]
q_start = [90.0 * math.pi/180.0
  , -15.4 * math.pi/180.0
  , 60 * math.pi/180.0
  , -200 * math.pi/180.0
  , 0 * math.pi/180.0 ]
'''

DEG2RAD = math.pi/180

GEAR_RATIO = [100.0, 88.0, 37.5, 2.0, 2.0]
MICROSTEP = [400, 400, 400, 400, 400]
JOINT_MIN_ANG = [-91*DEG2RAD, -15.49*DEG2RAD, -91*DEG2RAD, -201*DEG2RAD, -360*DEG2RAD]
JOINT_MAX_ANG = [ 91*DEG2RAD, 91*DEG2RAD, 61*DEG2RAD, 31*DEG2RAD,  360*DEG2RAD]
JOINT_MAX_VELO = [ 60*DEG2RAD,  60*DEG2RAD, 60*DEG2RAD, 90*DEG2RAD, 90*DEG2RAD]
'''
MOTOR_MIN_ANG = [-180*DEG2RAD, 0, -150*DEG2RAD, 0, -360*DEG2RAD]
MOTOR_MAX_ANG = [ 0,  120*DEG2RAD, 0,  260*DEG2RAD,  360**DEG2RAD]
MOTOR_MAX_VELO = [ 60*DEG2RAD,  60*DEG2RAD, 60*DEG2RAD, 90*DEG2RAD, 90*DEG2RAD]
'''
q_start = [90.0 * DEG2RAD
  , -15.4 * DEG2RAD
  , 60 * DEG2RAD
  , -200 * DEG2RAD
  , 0 * DEG2RAD ]
rad2freq_pulse = None

class MySerial():
  def __init__(self, _port_name, _baud_rate):
    '''
    self.ser = serial.Serial(port = _port_name
    , baudrate = _baud_rate
    , timeout = 0.01
    , writeTimeout = 0)
    if self.ser.isOpen():
      raise Exception('opened')
    '''
    self.ser = serial.Serial()
    self.ser.port = _port_name
    self.ser.baudrate = _baud_rate
    self.ser.timeout = 0.01
    self.ser.writeTimeout = 0
    
    self.ser.open()
    self.b_print = True
    self.serial_buf = ''

  def wait_start(self, name):
    print('wait start')
    for i in range(3):
      self.ser.write(b'who\n')
      t = time.time() + 1.0
      while time.time() < t:
        s = self.serial_read()
        if len(s)>0 and s[0:4]=='who:':
          _name = s[4:].split(' ')[0]
          if (type(name) is str and name==_name) or (type(name) is list and (_name in name) ):
            print('start')
            return
          else:
            raise Exception('wait_start() : wrong name : ' + _name + ' / ' + str(name))
    raise Exception('wait_start() : timeout')

  def serial_read(self):
    c = self.ser.read()
    while len(c)>0:
      if ord(c)<128:
        c = c.decode()
        if c=='\n' or c=='\r':
          if len(self.serial_buf)>0:
            b = self.serial_buf
            self.serial_buf = ''
            if self.b_print:
              print('READ : ' + b)
            return b
        else:
          self.serial_buf+= c
      c = self.ser.read()
    return ''

  def set_print(self, b_print):
    self.b_print = b_print
    
  def print_checksum(self, msg):
    sum = 0
    for i in range(len(msg)):
      sum+= ord(msg[i])
    cs = (sum & 127) + ord('0')
    msg2 = list(bytearray(msg.encode('ascii','ignore'))) + [cs, ord('\n')]
    self.ser.write(msg2)
#    print(bytearray(msg2).decode("ascii"))

  def set_gear_microstep(self, id, b_1_motor, gear=None, microstep=None):
    global GEAR_RATIO, MICROSTEP
    if gear is None:
      gear = GEAR_RATIO[id]
    if microstep is None:
      microstep = MICROSTEP[id]
    if b_1_motor:
      self.print_checksum('g%.3f %d ' % (gear, microstep))
    else:
      self.print_checksum('g%d %.3f %d ' % (id, gear, microstep))
    for i in range(5):
      t = time.time() + 1
      while time.time()<t:
        s = self.serial_read()
        if len(s)>2 and s[:2]=='g:':
          return
    raise Exception('set_gear_microstep() : timeout')

  def set_limit(self, id, b_1_motor, velo=None, min_ang=None, max_ang=None):
    global JOINT_MAX_VELO, JOINT_MIN_ANG, JOINT_MAX_ANG
    
    if velo is None:
      velo = joint2motor(JOINT_MAX_VELO)[id]
    else:
      JOINT_MAX_VELO[i] = velo
    if min_ang is None:
      min_ang = joint2motor(JOINT_MIN_ANG)[id]
    else:
      JOINT_MIN_ANG[i] = min_ang
    if max_ang is None:
      max_ang = joint2motor(JOINT_MAX_ANG)[id]
    else:
      JOINT_MAX_ANG[i] = max_ang
    
    velo = 999.0
    min_ang = -999.0
    max_ang = 999.0
    
    if b_1_motor:
      self.print_checksum('l%.3f %.3f %.3f ' % ( abs(velo), min_ang, max_ang))
    else:
      self.print_checksum('l%d %.3f %.3f %.3f ' % (id, abs(velo), min_ang, max_ang))
    for i in range(5):
      t = time.time() + 1
      while time.time()<t:
        s = self.serial_read()
        if len(s)>2 and s[:2]=='l:':
          return
    raise Exception('set_limit() : timeout')

  def set_target(self, time, positions, b_1_motor):
    global JOINT_MIN_ANG, JOINT_MAX_ANG
    for i in range(len(positions)):
      if positions[i]>JOINT_MAX_ANG[i] or positions[i]<JOINT_MIN_ANG[i]:
        raise Exception('angle exceeds the limit [%d] : %f , %f / %f' % \
          (i, positions[i], JOINT_MIN_ANG[i], JOINT_MAX_ANG[i]))

    q_motor = joint2motor(positions)
#    q_motor[2] = -q_motor[2]
    if b_1_motor:
      cmd = 'p%.3f %.3f ' % (time, q_motor[0])
    else:
      cmd = 'p%.3f ' % (time)
      for q in q_motor:
        cmd+= '%.3f ' % (q)

    self.print_checksum(cmd)
    print('set_target() pos : ' + str(positions))
    print('set_target() : ' + cmd)
  
  
  def reset_arduino(self):
    self.ser.write(b'\nreset\n')
    t = time.time()
    
    while 1:
      s = self.serial_read()
      if len(s)>0 and s=='reset:ok':
        break
      if time.time() - t > 1.0:
        raise Exception('reset timeout : ' + self.ser.port)
        
  def reset_joint_state(self):
    print('reset_joint_state')
    try:
      srv_name = 'affbot/joint_state_publisher/set_zero'
      rospy.wait_for_service(srv_name, timeout=0.01)
      j_zero = rospy.ServiceProxy(srv_name, AffbotSetZero)
      j_zero()
    except rospy.ROSException:
      print('No joint_state_publisher found')

def get_arduinos(port_prefix, max_num, baud_rate, who):
  sers = []
  for i in range(9):
    ser = None
    try:
      port = port_prefix + str(i)
      ser = MySerial(port, baud_rate)
      ser.wait_start(who)
      sers.append(ser)
      if len(sers)==max_num:
        return sers
      print(port + ' : ok')
    except (serial.serialutil.SerialException, Exception) as e:
      print(port + ' : fail : ' + str(e))
      if ser is not None:
        ser.ser.close()
  for ser in sers:
    ser.ser.close()
  raise Exception('Too few arduino found : %d' % (len(sers)))
  

def motor2joint(q_motor, add_q_start=True):
  global q_start
  if len(q_motor)!=5:
    raise Exception('motor2joint : joint num is not 5 (%d)' % (len(q_motor)))
  q = [0,0,0,0,0]
  q[0] = q_motor[0]
  q[1] = q_motor[1]
#  q[2] = q_motor[2] - q[1]
  q[2] = -q_motor[2] - q[1] # reverse direction
  q[3] = (q_motor[3] + q_motor[4]) * 0.5 - q[1] - q[2]
  q[4] = (q_motor[3] - q_motor[4]) * 0.5
  '''
  q[0] = q_motor[0] / GEAR_RATIO[0]
  q[1] = q_motor[1] / GEAR_RATIO[1]
  q[2] = q_motor[2] / GEAR_RATIO[2] - q[1]
  q[3] = (q_motor[3] + q_motor[4]) / (2.0 * GEAR_RATIO[3] ) - q[1] - q[2]
  q[4] = (q_motor[3] - q_motor[4]) / (2.0 * GEAR_RATIO[4] )
  '''
  if add_q_start:
    for i in range(5):
      q[i]+= q_start[i]
  return q


def joint2motor(q):
  global GEAR_RATIO, q_start

  q_link = [0,0,0,0,0]
  for i in range(5):
    q_link[i] = q[i] - q_start[i]

  q_motor = [0,0,0,0,0]
  '''
  q_motor[0] = q_link[0] * GEAR_RATIO[0]
  q_motor[1] = q_link[1] * GEAR_RATIO[1]
  q_motor[2] = (q_link[1] + q_link[2]) * GEAR_RATIO[2]
  q_motor[3] = (q_link[1] + q_link[2] + q_link[3] + q_link[4]) * GEAR_RATIO[3]
  q_motor[4] = (q_link[1] + q_link[2] + q_link[3] - q_link[4]) * GEAR_RATIO[4]
  '''
  q_motor[0] = q_link[0]
  q_motor[1] = q_link[1]
  q_motor[2] = (q_link[1] + q_link[2])
  q_motor[3] = (q_link[1] + q_link[2] + q_link[3] + q_link[4])
  q_motor[4] = (q_link[1] + q_link[2] + q_link[3] - q_link[4])
  # have to switch q[3] and q[4] ?
#  q_motor[3] = (q_link[1] + q_link[2] + q_link[3] - q_link[4])
#  q_motor[4] = (q_link[1] + q_link[2] + q_link[3] + q_link[4])  

  # reverse direction
  q_motor[2]*= -1.0
  return q_motor
  

def get_rad2freq_pulse():
  global rad2freq_pulse, MICROSTEP, GEAR_RATIO
  if rad2freq_pulse is None:
    r = []
    for i in range(5):
      r.append( MICROSTEP[i] * GEAR_RATIO[i] / (2*math.pi) )
    rad2freq_pulse = r
  return rad2freq_pulse

