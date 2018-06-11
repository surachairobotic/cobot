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
#import keyboard
import time

fname = 'pos_calib.txt'
b_save = False


class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

'''
def key_press(key):
  global pos
  if key.name=='s':
    b_save = True
  elif key.name=='n':
    with open(fname, 'at') as f:
      f.write('--\n')
'''

def callback_joint(joints):
  global fname, b_save
  if b_save:
    with open(fname, 'at') as f:
      s = ''
      for p in joints.position:
        s+= str(p)
      f.write(s + '\n')
      print(s)

if __name__ == "__main__":
  try:
    try:
      os.remove(fname)
    except:
      pass
    rospy.init_node('record_pos')
    sub_joint = rospy.Subscriber("/joint_states", JointState, callback_joint)
#    keyboard.on_press(key_press)
    getch = _Getch()
    while not rospy.is_shutdown():
      c = getch()
      print(c)
      if c=='q':
        break
      elif c=='s':
        b_save = True
      elif c=='c':
        with open(fname, 'at') as f:
          f.write('--\n')
      rospy.sleep(0.01)
  except KeyboardInterrupt:
    print('SIGINT')
