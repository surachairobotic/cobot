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

GEAR_RATIO = [(400.0*100)/1600.0
 ,(400.0*88.0)/1600.0
 ,(400.0*37.5)/1600.0
 ,(400.0*2.0)/1600.0
 ,(400.0*2.0)/1600.0]
q_start = [90.0 * math.pi/180.0
  , -15.4 * math.pi/180.0
  , 60 * math.pi/180.0
  , -200 * math.pi/180.0
  , 0 * math.pi/180.0 ]
  
def motor2joint(q_motor, add_q_start=True):
  global GEAR_RATIO, q_start
  if len(q_motor)!=5:
    raise Exception('motor2joint : joint num is not 5 (%d)' % (len(q_motor)))
    
  '''
  q(0) = q_motor(0) / GEAR_RATIO(0)
	q(1) = q_motor(1) / GEAR_RATIO(1)
	q(2) = q_motor(2) / GEAR_RATIO(2) - q(1)
	q(3) = (q_motor(3) + q_motor(4)) / (2.0 * GEAR_RATIO(3) ) - q(1) - q(2)
	q(4) = (q_motor(3) - q_motor(4)) / (2.0 * GEAR_RATIO(4) )
	
	For i=0 To 4
		q(i) = q(i) + q_start(i)
	Next
	'''
  q = [0,0,0,0,0]
  q[0] = q_motor[0] / GEAR_RATIO[0]
  q[1] = q_motor[1] / GEAR_RATIO[1]
  q[2] = q_motor[2] / GEAR_RATIO[2] - q[1]
  q[3] = (q_motor[3] + q_motor[4]) / (2.0 * GEAR_RATIO[3] ) - q[1] - q[2]
  q[4] = (q_motor[3] - q_motor[4]) / (2.0 * GEAR_RATIO[4] )
  
  if add_q_start:
    for i in range(5):
      q[i]+= q_start[i]
  return q
  

def joint2motor(q):
  global GEAR_RATIO, q_start
  '''
  q_motor(0) = q_link(0) * GEAR_RATIO(0)
	q_motor(1) = q_link(1) * GEAR_RATIO(1)
	q_motor(2) = (q_link(1) + q_link(2)) * GEAR_RATIO(2)
	q_motor(3) = (q_link(1) + q_link(2) + q_link(3) + q_link(4)) * GEAR_RATIO(3)
	q_motor(4) = (q_link(1) + q_link(2) + q_link(3) - q_link(4)) * GEAR_RATIO(4)
	'''
  q_link = [0,0,0,0,0]
  for i in range(5):
    q_link[i] = q[i] - q_start[i]
  
  q_motor = [0,0,0,0,0]
  q_motor[0] = q_link[0] * GEAR_RATIO[0]
  q_motor[1] = q_link[1] * GEAR_RATIO[1]
  q_motor[2] = (q_link[1] + q_link[2]) * GEAR_RATIO[2]
  q_motor[3] = (q_link[1] + q_link[2] + q_link[3] + q_link[4]) * GEAR_RATIO[3]
  q_motor[4] = (q_link[1] + q_link[2] + q_link[3] - q_link[4]) * GEAR_RATIO[4]

  return q_motor
	
	
  
  
  
  
    
