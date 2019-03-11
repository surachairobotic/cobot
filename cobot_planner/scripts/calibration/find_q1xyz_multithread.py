#!/usr/bin/env python
# -*- coding: utf-8 -*-

# lib for calibrate_chess.py

import sys

sys.path.insert(0,'Z:\\Git\\cobot\\cobot_planner\\scripts')
sys.path.insert(0,'/home/tong/catkin_ws/src/cobot/cobot_planner/scripts')

import os
from time import sleep
import math
import numpy as np
import time
import eq
import set_const
import copy
import multiprocessing as mp
from find_err_multithread import get_R, rx, ry, rz

def find_q1xyz_thread(get_xyz, pos, xyz_ref, off_q, off_x, off_bq, dq, dqmax, dx, dxmax, dbq, dbqmax, thread_range, results):
  best_score = 999999999.0
  nq = int(dqmax*2 / dq)
  nx = int(dxmax*2 / dx)
  nbq = int(dbqmax*2 / dbq)
  
  if nx==0:
    nx = 1
    
  q = copy.deepcopy(off_q)
  xyz = np.array([0.0]*3)
  
  param = np.array([0.0]*12)
  bq = param[0:3]
  xyz = param[9:12]
  q = param[3:9]
  for i in range(6):
    q[i] = off_q[i]
  
  cnt = 0
  n_all = 1.0/float( (thread_range[1]-thread_range[0])*nx**3*nbq**2 )
  with open('./results/calib_result_q1xyz_%d_%d.txt' % (thread_range[0], thread_range[1]), 'at') as f:
  
    for q1 in range(thread_range[0], thread_range[1]):
      q[0] = dq*q1 - dqmax + off_q[0]
      for bqx in range(nbq):
        bq[0] = dbq*bqx - dbqmax + off_bq[0]
        Rx = rx( bq[0] )
        for bqy in range(nbq):
          bq[1] = dbq*bqy - dbqmax + off_bq[1]
          Ry = ry( bq[1] )
          R = Ry.dot(Rx)
          
          '''
          for x in range(nx):
            xyz[0] = dx*x - dxmax + off_x[0]
            for y in range(nx):
              xyz[1] = dx*y - dxmax + off_x[1]
              for z in range(nx):
                xyz[2] = dx*z - dxmax + off_x[2]
          '''
          score = 0.0
          cnt+=1
          for i in range(len(pos)):
            for p2 in pos[i]:
              #e = get_xyz(p2[0:6]+q) + xyz - xyz_ref[i]
              e = R.dot( get_xyz(p2[0:6]+q) ) + xyz - xyz_ref[i]
              for j in range(3):
                score+= e[j]**2
          score = math.sqrt( score/(len(pos)*len(pos[0])) )
          if score < best_score:
            best_score = score
            best_q = copy.deepcopy(q)
            best_xyz = copy.deepcopy(xyz)
            best_bq = copy.deepcopy(bq)
            
            s = '[%d:%d] %.2f : %.6f :  [%f, %f, %f, %f, %f, %f],[%f, %f, %f],[%f, %f]' % (thread_range[0],thread_range[1], cnt*n_all, best_score, q[0], q[1], q[2], q[3], q[4], q[5], xyz[0], xyz[1], xyz[2], bq[0], bq[1])
            print(s)
            f.write( s + '\n' )

  print('[%d:%d] end' % (thread_range[0],thread_range[1]))
  results.put([best_q, best_xyz, best_bq, best_score])


def find_q1xyz_multithread(get_xyz, pos, xyz_ref, off_q, off_x, off_bq, dq, dqmax, dx, dxmax, dbq, dbqmax):
  best_score = 999999999.0
  nq = int(dqmax*2 / dq)
  nx = int(dxmax*2 / dx)
#  q = np.array([0.0]*len(off_q))
#  xyz = np.array([0.0]*3)
  
  n_thread = 8
  if n_thread>nq:
    n_thread = nq
  
  ranges = []
  n = 0
  for i in range(n_thread):
    n1 = int(math.ceil( (nq-n) / float(n_thread-i) ))
    ranges.append([n,n+n1])
    n+= n1
  assert( ranges[-1][1]==nq )
  ths = []
  results = []

  queues = []
  for r in ranges:
    queue = mp.Queue()
    ths.append( mp.Process(target=find_q1xyz_thread
      , args=([get_xyz, pos, xyz_ref, off_q, off_x, off_bq, dq, dqmax, dx, dxmax, dbq, dbqmax, r, queue])) )
    queues.append(queue)

  for th in ths:
    th.start()
  while len(results)!=n_thread:
    for q in queues:
      r = q.get()
      if r:
        results.append(r)
    time.sleep(1.0)
  best_r = None
  for r in results:
    print(r)
    if best_r is None:
      best_r = r
    else:
      if best_r[3]>r[3]:
        best_r = r
#  print(best_r)
  print('score : '+str(best_r[3]))
  return best_r[0:3]






