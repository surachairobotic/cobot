#!/usr/bin/env python
# -*- coding: utf-8 -*-

# calibrate robot's joints using file from record_pos.py

import sys
import os
'''
import rospy
from sensor_msgs.msg import JointState
'''
from time import sleep
import math
import numpy as np
import time
from moveit_msgs.srv import GetPositionFK

fname = 'pos_calib.txt'

compute_fk = None
def find_err(pos, dq, dqmax):
  global compute_fk
  header = Header()
  header.frame_id = 'base_link'
  robot_state = RobotState()
  robot_state.header = header
  joint_names = [('J'+str(i+1)) for i in range(6)]
  robot_state.joint_state.name = joint_names

  best_score = 999999999.0
  n_loop = (dqmax*2 / dq)**6
  q = np.array([0]*6)
  for q1 in range(n):
    q[0] = dq*q1 - dqmax
    for q2 in range(n):
      q[1] = dq*q2 - dqmax
      for q3 in range(n):
        q[2] = dq*q3 - dqmax
        for q4 in range(n):
          q[3] = dq*q4 - dqmax
          for q5 in range(n):
            q[4] = dq*q5 - dqmax
            for q6 in range(n):
              q[5] = dq*q6 - dqmax
              score = 0
              for j in range(pos):
                pos2 = []
                for k in range(pos[j]):
                  for l in range(len(pos))
                  c = compute_fk(header, ['tool0'], robot_state)
                  if c.error_code.val==1:
                    p = c.pose_stamped[0].pose.position
                    pos2.append([p.x,p.y,p.z])
                pos2 = np.array(pos2)
                score+= np.var(pos2[:,0]) + np.var(pos2[:,1]) + np.var(pos2[:,2])
              if score < best_score:
                best_score = score
                best_q = copy.deepcopy(q)
                print(str(best_score) + ' : ' + str(best_q))
  return best_q

if __name__ == "__main__":
  try:
    pos = []
    with open(fname, 'rt') as f:
      p = []
      while True:
        s = f.readline().strip()
        if not s:
          break
        elif s=='--':
          if len(p)>0:
            pos.append(p)
            p = []
        else:
          a = s.split(' ')
          if len(a)!=6:
            print('Invalid string : ' + s)
            exit()
          p.append([float(b) for b in a])
      if len(p)>0:
        pos.append(p)
    #print(pos)
    rospy.wait_for_service('compute_fk')
    compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    find_err(pos, 0.2, 3.0)
    c = compute_fk(header, ['tool0'], robot_state)
  except KeyboardInterrupt:
    print('SIGINT')
