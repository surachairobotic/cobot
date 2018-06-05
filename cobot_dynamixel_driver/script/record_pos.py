#!/usr/bin/env python
# -*- coding: utf-8 -*-

# record tip's positions and joint angles for calibration

import sys
import os
import rospy
from sensor_msgs.msg import JointState
from time import sleep
import math
import numpy as np
import keyboard
import time

fname = 'pos_calib.txt'
b_save = False
def key_press(key):
  global pos
  if key.name=='s':
    b_save = True
  elif key.name=='n':
    with open(fname, 'at') as f:
      f.write('--\n')


def callback_joint(joints):
  global fname, b_save
  if b_save:
    with open(fname, 'at') as f:
      for p in joints.position:
        f.write('%f ')
      f.write('\n')

if __name__ == "__main__":
  try:
    try:
      os.remove(fname)
    except:
      pass
    rospy.init_node('affbot_controller')
    sub_joint = rospy.Subscriber("/joint_states", JointState, callback_joint)
    keyboard.on_press(key_press)
    while not rospy.is_shutdown():
      rospy.sleep(0.01)
  except KeyboardInterrupt:
    print('SIGINT')
