#!/usr/bin/env python

import sys
import os
import rospy

import math
import time
import actionlib
import tf
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Vector3, Point
from actionlib_msgs.msg import GoalStatus, GoalID
from cobot_pick.msg import CobotFindObjectAction, CobotFindObjectGoal
from visualization_msgs.msg import Marker

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

from scipy.spatial.transform import Rotation as R

from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse

# Global Declarations
state = 0
action_client_object = None
action_client_basket = None
goal_poses = PoseArray()
b_jntArm = False
box = String()
p1 = Point()
p2 = Point()
p3 = Point()
p4 = Point()
pp1 = Marker()
pp2 = Marker()
pp3 = Marker()
jnt = JointState()
robot_state = RobotState()
pub_box_marker = rospy.Publisher("/cobot/marker/box", Marker, queue_size=10)

def tg_callback(msg):
  global state
  if state == 0:
    if msg.data is True:
      rospy.loginfo("tg_callback is True")
      state = 1
  else:
    state = 0

def done_cb_obj(_state, result):
  global state
  if state is 1:
    state = 2

def feedback_cb_obj(feedback):
  rospy.loginfo('camera obj feedback : ' + str(feedback))

def active_cb_obj():
  rospy.loginfo('camera obj is active')

def done_cb_basket(_state, result):
  global state

def feedback_cb_basket(feedback):
  rospy.loginfo('camera basket feedback : ' + str(feedback))

def active_cb_basket():
  rospy.loginfo('camera basket is active')

def shutdown_cb():
  global action_client_object, action_client_basket
  action_client_object.cancel_goal()
  action_client_basket.cancel_goal()

def jnt_callback(msg):
  global jnt
  jnt = msg

def pick_marker(msg, aa):
  global pub_box_marker, pp
  rospy.loginfo('pick_marker_callback')
  p1.x = msg.position.x
  p1.y = msg.position.y
  p1.z = msg.position.z

  a2 = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
  a2 = a2.as_dcm()
  print(a2)
  a3 = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
  a3 = a3.as_dcm()
  a4 = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
  a4 = a4.as_dcm()

  p2.x = a2[0][0]*0.05 + p1.x;
  p2.y = a2[1][0]*0.05 + p1.y;
  p2.z = a2[2][0]*0.05 + p1.z;

  p3.x = a3[0][1]*0.05 + p1.x;
  p3.y = a3[1][1]*0.05 + p1.y;
  p3.z = a3[2][1]*0.05 + p1.z;

  p4.x = p1.x - a4[0][2]*0.05;
  p4.y = p1.y - a4[1][2]*0.05;
  p4.z = p1.z - a4[2][2]*0.05;

  pp1.header.frame_id = "world"
  pp1.header.stamp = rospy.Time.now()
  pp1.action = pp1.ADD
  pp1.type = pp1.ARROW
  pp1.id = 0+aa

  pp1.points = []
  pp1.points.append(p2);
  pp1.points.append(p1);

  pp1.scale.x = 0.01
  pp1.scale.y = 0.01
  pp1.scale.z = 0.01

  pp1.color.a = 1.0
  pp1.color.r = 1.0
  pp1.color.g = 0.0 
  pp1.color.b = 0.0
  pub_box_marker.publish(pp1)

  pp2.header.frame_id = "world"
  pp2.header.stamp = rospy.Time.now()
  pp2.action = pp2.ADD
  pp2.type = pp2.ARROW
  pp2.id = 1+aa

  pp2.points = []
  pp2.points.append(p3);
  pp2.points.append(p1);

  pp2.scale.x = 0.01
  pp2.scale.y = 0.01
  pp2.scale.z = 0.01

  pp2.color.a = 1.0
  pp2.color.r = 0.0
  pp2.color.g = 1.0 
  pp2.color.b = 0.0
  pub_box_marker.publish(pp2)

  pp3.header.frame_id = "world"
  pp3.header.stamp = rospy.Time.now()
  pp3.action = pp3.ADD
  pp3.type = pp3.ARROW
  pp3.id = 2+aa

  pp3.points = []
  pp3.points.append(p4);
  pp3.points.append(p1);

  pp3.scale.x = 0.01
  pp3.scale.y = 0.01
  pp3.scale.z = 0.01

  pp3.color.a = 1.0
  pp3.color.r = 0.0
  pp3.color.g = 0.0 
  pp3.color.b = 1.0
  pub_box_marker.publish(pp3)
   
if __name__ == "__main__":
  try:
    rospy.init_node('camera_marker', anonymous=True)
    rospy.loginfo('camera_marker . . .')
    sub_tg = rospy.Subscriber("/cobot/trigger", Bool, tg_callback)
    sub_jnt = rospy.Subscriber("/joint_state", JointState, jnt_callback)

    global state, action_client_object, action_client_basket, jnt

    rospy.loginfo('waiting camera server')
    action_client_object = actionlib.SimpleActionClient('/cobot/find_object', CobotFindObjectAction)
    action_client_basket = actionlib.SimpleActionClient('/cobot/find_basket', CobotFindObjectAction)
    action_client_object.wait_for_server()
    action_client_basket.wait_for_server()
    obj = CobotFindObjectGoal()
    basket = CobotFindObjectGoal()
    rospy.on_shutdown(shutdown_cb)
    rospy.loginfo('camera server : ok')

    rospy.loginfo('waiting GetPositionFK server')
    rospy.wait_for_service('compute_fk')
    getFk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    rospy.loginfo('GetPositionFK server : ok')

    old_state = -1
    while not rospy.is_shutdown():
      if old_state != state:
        rospy.loginfo("state : %d", state)
        old_state = state
        if state is 1:
          rospy.loginfo('send action camera')
          action_client_object.send_goal(goal=obj
            , done_cb = done_cb_obj  # Callback that gets called on transitions to Done
            , active_cb = active_cb_obj  # No-parameter callback that gets called on transitions to Active
            , feedback_cb = feedback_cb_obj  # Callback that gets called whenever feedback for this goal is received
          )
          action_client_basket.send_goal(goal=basket
            , done_cb = done_cb_basket  # Callback that gets called on transitions to Done
            , active_cb = active_cb_basket  # No-parameter callback that gets called on transitions to Active
            , feedback_cb = feedback_cb_basket  # Callback that gets called whenever feedback for this goal is received
          )
        elif state is 2:
          rospy.loginfo("camera response")
          if len(action_client_object.get_result().labels) > 0:
            for i in range(len(action_client_object.get_result().labels)):
              print(action_client_object.get_result().labels[i])
              if action_client_object.get_result().labels[i].find('M1') != -1:
                pick_marker(action_client_object.get_result().poses[i], 0)
                res = getFk(robot_state.joint_state.header, ['tool0'], robot_state)
                pick_marker(res.pose_stamped[0].pose, 3)
                break
          state = 0
      rospy.sleep(0.01)
  finally:
    sub_tg.unregister()
    rospy.loginfo('end')
