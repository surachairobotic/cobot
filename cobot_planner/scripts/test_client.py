#!/usr/bin/env python

import sys
import os
import rospy
import math
from geometry_msgs.msg import Pose
from cobot_planner.srv import *
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


srv_planning = None
joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


def get_pose(ang):
  global joint_names, compute_fk
  header = Header()
  header.frame_id = 'base_link'
  robot_state = RobotState()
  robot_state.joint_state.name = joint_names
  robot_state.joint_state.position = ang
  c = compute_fk(header, ['tool0'], robot_state)
  if c.error_code.val!=1:
    raise Exception('compute_fk error : error_code = ' + str(c.error_code.val))
  return c.pose_stamped[0].pose



def planning(start_pose, end_pose):
  global srv_planning, joint_names
  try:
#    req = CobotPlanningRequest()
    res = srv_planning(joint_names=joint_names
    , start_joints=[0,0,0,0,0]
    , end_joints=[0.2,0.3,0.7,0.2,0.1]
    , start_pose=start_pose
    , end_pose=end_pose
    , type="line"
    , max_velocity=0.5
    , max_acceleration=3
    , step_time=0.1)
    return res
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e



if __name__ == "__main__":
  print("waiting 'cobot_planning'")
  rospy.wait_for_service('cobot_planning')
  srv_planning = rospy.ServiceProxy('cobot_planning', CobotPlanning)
  print("waiting 'compute_fk'")
  rospy.wait_for_service('compute_fk')
  compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
  print("waiting 'compute_ik'")
  rospy.wait_for_service('compute_ik')
  compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
  
  
  print('start')
  start_pose = get_pose([0,0,0,0,0,0])
  end_pose = get_pose([0.0,0.0,0.3,0.2,0.1,0])
  res = planning(start_pose, end_pose)
  if res is not None:
    if res.error_code==0:
      with open('plan.txt', 'wt') as f:
        for p in res.points:
          s = str(p.time_from_start.to_sec()) + ' '
          for v in p.positions:
            s+= str(v) + ' '
          for v in p.velocities:
            s+= str(v) + ' '
          for v in p.accelerations:
            s+= str(v) + ' '
          f.write(s + '\n')
      #plot_plan.plot(res.points)
    else:
      print('error : '+str(res.error_code))
    
  
