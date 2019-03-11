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

get_xyz = None


def find_q2q5_thread(pos, off, dq, dqmax, thread_range, results):
  global get_xyz
  best_score = 999999999.0
  nq = int(dqmax*2 / dq)
  q = np.array(copy.deepcopy(off))
  
  cnt = 0
  n_all = 1.0/float( (thread_range[1]-thread_range[0])*nq**3 )
  with open('./results/calib_result_q2q5_%d_%d.txt' % (thread_range[0], thread_range[1]), 'at') as f:
    for q2 in range(thread_range[0], thread_range[1]):
      q[1] = dq*q2 - dqmax + off[1]
      for q3 in range(nq):
        q[2] = dq*q3 - dqmax + off[2]
        for q4 in range(nq):
          q[3] = dq*q4 - dqmax + off[3]
          for q5 in range(nq):
            q[4] = dq*q5 - dqmax + off[4]
            score = 0.0
            cnt+=1
            for p1 in pos:
              pos2 = []
              for p2 in p1:
                pos2.append(get_xyz(p2[0:6]+q))
                #for l in range(6):
                #  rb_pos[l] = p2[l] + q[l]
                #pos2.append(get_xyz(rb_pos))
              pos2 = np.array(pos2)
              score+= np.var(pos2[:,0]) + np.var(pos2[:,1]) + np.var(pos2[:,2])
            if score < best_score:
              best_score = score
              best_q = copy.deepcopy(q)
              s = '[%d:%d] %.2f : %.6f :  %f %f %f %f %f %f' % (thread_range[0],thread_range[1], cnt*n_all, best_score, q[0], q[1], q[2], q[3], q[4], q[5])
              print(s)
              f.write( s + '\n' )
#              print(str(cnt*n_all) + ' : ' + str(best_score) + ' : ' + str(best_q))
#              f.write( '%.8f %f %f %f %f %f %f\n' % (best_score \
#                , q[0], q[1], q[2], q[3], q[4], q[5]))
  print('[%d:%d] end' % (thread_range[0],thread_range[1]))
  results.put([best_q, best_score])
  return best_q



def find_q2q5_multithread(_get_xyz, pos, off_q, dq, dqmax):
  global get_xyz
  best_score = 999999999.0
  nq = int(dqmax*2 / dq)
#  q = np.array([0.0]*len(off_q))
  
  get_xyz = _get_xyz

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
    ths.append( mp.Process(target=find_q2q5_thread
      , args=([pos, off_q, dq, dqmax, r, queue])) )
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
      if best_r[1]>r[1]:
        best_r = r
#  print(best_r)
  print('score : '+str(best_r[1]))
  return best_r[0]






