#!/usr/bin/env python

# get data from driver for plotting

import sys
import os
import math
import rospy
from sensor_msgs.msg import JointState
import copy

sub = None
b_callback = False
fname = 'data.txt'
t_start = None
def callback_joint(js):
  global b_callback, sub, fname, t_start
  b_callback = True
  b_error = True
  try:
    if t_start is None:
      t_start = rospy.Time.now()
    with open(fname, 'at') as f:
      s = str((js.header.stamp - t_start).to_sec())
      #for v in js.name:
      #  s+= ' ' + str(v)
      for v in js.position:
        s+= ' ' + str(v)
      for v in js.velocity:
        s+= ' ' + str(v)
      for v in js.effort:
        s+= ' ' + str(v)
      f.write(s+'\n')
    b_error = False
  finally:
    if b_error and sub is not None:
      sub.unregister()
      sub = None
    b_callback = False

if __name__ == "__main__":
  rospy.init_node('get_data', anonymous=True)
  try:
    os.remove(fname)
  except:
    pass
  try:
    sub = rospy.Subscriber("/joint_states", JointState, callback_joint)
    print('start ...')
    while sub is not None and not rospy.is_shutdown():
      rospy.sleep(0.01)
    if sub is not None:
      sub.unregister()
      while b_callback:
        rospy.sleep(0.01)
    rospy.loginfo("end")
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
  except rospy.exceptions.ROSInterruptException:
    pass
  print('end')
