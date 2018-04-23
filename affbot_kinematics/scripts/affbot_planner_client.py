#!/usr/bin/env python

import sys
import os
import rospy
import math
import my_kinematics as kinematics
import plot_plan
import copy
import numpy as np
from affbot_kinematics.srv import *
from geometry_msgs.msg import Pose
from affbot_kinematics.msg import *

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import lib_controller

srv_planning = None
joint_names = kinematics.joint_names


def planning(start_pose, end_pose):
  global srv_planning, joint_names
  try:
    res = srv_planning(joint_names=[] #joint_names
    , start_joints=[]
    , end_joints=[]
    , start_pose=start_pose
    , end_pose=end_pose
    , type="p2p"
    , max_velocity=0.7
    , max_acceleration=0.5
    , step_time=0.1)
    return res
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


if __name__ == "__main__":
  rospy.init_node('affbot_planner_client')
  print("waiting 'affbot_planning'")
  rospy.wait_for_service('affbot/planner/planning')
  srv_planning = rospy.ServiceProxy('affbot/planner/planning', AffbotPlanning)
  kinematics.init()
  
  print('start')
#  start_pose = kinematics.get_pose([1.5707963267948966, -0.2687807048071268, 1.0471975511965976, -3.490658503988659, 0.0])
#  start_pose = kinematics.get_current_pose()
  start_joints = kinematics.get_current_joints()
  start_pose = kinematics.get_pose(start_joints)
#  start_pose.position.x -= 0.20

  '''
  end_joints = list(copy.deepcopy(start_joints))
  end_joints[0]+= math.pi*0.49
  end_pose = kinematics.get_pose(end_joints)
  '''
  
#  end_pose = copy.deepcopy(start_pose)
#  end_pose.position.x-= 0.2
#  end_pose.position.z+= 0.2
  
  '''
  end_joints = list(copy.deepcopy(start_joints))
  end_motors = lib_controller.joint2motor(end_joints)
  end_motors[4]+= 0.6
  end_joints = lib_controller.motor2joint(end_motors)
  end_pose = kinematics.get_pose(end_joints)
  '''
  
  '''
  end_joints = list(kinematics.get_joints(start_pose))
  end_joints[4]+= 0.4
  end_pose = kinematics.get_pose(end_joints)
  '''
  
  '''
  
  end_joints = kinematics.get_joints(start_pose)
  end_motors = lib_controller.joint2motor(end_joints)
  end_motors[3]-=10.0
  end_joints2 = lib_controller.motor2joint(end_motors)
  print(end_joints)
  print(end_joints2)
  end_pose = kinematics.get_pose(end_joints2)
  '''
  
#  end_pose = kinematics.get_pose([0,0,0,0,0])
#  end_pose.position.x-= 0.40

  end_pose = kinematics.get_pose([1.5707963267948966, -0.2687807048071268, 1.0471975511965976, -3.490658503988659, 0.0])
  
  print(start_pose)
  print(end_pose)
#  exit()

  res = planning(start_pose, end_pose)
  if res.error_code==0:
    '''
    rospy.logwarn(start_joints)
    rospy.logwarn(res.points[1].positions)
    rospy.logwarn(np.array(start_joints) - np.array(res.points[1].positions))
    for i in range(len(res.points)):
      print(res.points[i].positions)
    for i in range(len(start_joints)):
      if abs(start_joints[i] - res.points[0].positions[i]) > 0.2:
        rospy.logwarn('start joint does not match')
        rospy.logwarn(start_joints)
        rospy.logwarn(res.points[0].positions)
        rospy.logwarn(np.array(start_joints) - np.array(res.points[0].positions))
        exit()
    '''
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
    
  
