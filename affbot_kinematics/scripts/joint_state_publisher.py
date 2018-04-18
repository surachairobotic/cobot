#!/usr/bin/env python

import sys
import os
import math
import time
import serial
import rospy
import copy
import lib_controller
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from affbot_kinematics.srv import AffbotSetZero


joint_name = ['J1', 'J2', 'J3', 'J4', 'J5']
motor_position = [0.0,0.0,0.0,0.0,0.0]
motor_velocity = [0.0,0.0,0.0,0.0,0.0]
joint_effort = [0.0,0.0,0.0,0.0,0.0]
sers = []
last_plan = None
b_set_zero = False
b_save_file = False
file_name = 'joint_state.txt'

def set_zero(req):
  global b_set_zero
  b_set_zero = True
  return []


if __name__ == "__main__":
  rospy.init_node('affbot_joint_state_publisher')
  '''
  if len(sys.argv)<=1:
    print('no port num found : ')
    exit()
  '''
  if len(sys.argv)>1:
    if sys.argv[1]=='save':
      b_save_file = True
      try:
        os.remove(file_name)
      except:
        pass
  
  
  try:
    # init serial
    sers = lib_controller.get_arduinos("/dev/ttyACM", 3, 115200, 'get_pulse_due')
    print('arduino found : %d' % len(sers))
    '''
    for i in range(len(sys.argv)-1):
      sers.append(lib_controller.MySerial("/dev/ttyACM" + sys.argv[i+1], 115200))
#      sers[-1].set_print(False)
    print('waiting start')
    try:
      for i in range(len(sers)):
        sers[i].wait_start('get_pulse_due')
        sers[i].reset_arduino()
    except serial.serialutil.SerialException:
      exit()
    '''
      
    # init ros
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    srv = rospy.Service('affbot/joint_state_publisher/set_zero', AffbotSetZero, set_zero)
    
      
    print('start')
    t_start = rospy.Time.now()
    while not rospy.is_shutdown():
      time.sleep(0.01)
      if b_set_zero:
        for k in range(len(sers)):
          sers[k].reset_arduino()
        b_set_zero = False
        print('reset ok')
      for k in range(len(sers)):
        ser = sers[k]
        s = ser.serial_read()
        if len(s)>3:
#          print(s)
          if s[0]=='<' and s[-1]=='>':
            start_id = int(s[1])
            arr = s[3:-2].split(' ')
            if len(arr)%2!=0:
              print('Invalid data len (%d): %s' %( len(arr), s ))
              continue
            n_joint = len(arr) / 2
            if n_joint>2:
              print('Invalid join num (%d): %s' %( n_joint, s ))
              continue
            '''
            if len(arr)==4:
              n_joint = 2
              start_id = 0
            elif len(arr)==6:
              n_joint = 3
              start_id = 2
            else:
              print('Invalid data len (%d): %s' %( len(arr), s ))
              continue
            '''
            try:
              j = JointState()
              j.header.stamp = rospy.Time.now()
              j.header.frame_id = 'base_link'
              j.name = joint_name
              
              for i in range(n_joint):
                r = 1000.0 / (lib_controller.MICROSTEP[start_id + i] 
                  * lib_controller.GEAR_RATIO[start_id + i])
                motor_position[start_id + i] = float(arr[i*2]) * r
                motor_velocity[start_id + i] = float(arr[i*2+1]) * r
              joint_position = lib_controller.motor2joint( motor_position )
              joint_velocity = lib_controller.motor2joint( motor_velocity, add_q_start=False )
              if b_save_file:
                with open(file_name, 'at') as f:
                  f.write('%f' % (j.header.stamp - t_start).to_sec())
                  for i in range(len(motor_position)):
                    f.write(' %f %f %f %f' % (motor_position[i], motor_velocity[i]
                      , joint_position[i], joint_velocity[i]))
                  f.write('\n')
              
              j.position = joint_position
              j.velocity = joint_velocity
              j.effort = joint_effort
              pub.publish(j)
            except ValueError:
              print('Invalid data vale : ' + s)
    print('end')
  #except serial.serialutil.SerialException:
  #  print('serial break')
  except KeyboardInterrupt:
    print('SIGINT')
  finally:
    for ser in sers:
      ser.ser.close()
    
