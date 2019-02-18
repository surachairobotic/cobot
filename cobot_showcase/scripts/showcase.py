#!/usr/bin/env python

import sys
import os
import rospy
from std_msgs.msg import Bool

import time
import actionlib
from geometry_msgs.msg import Pose, PoseArray
from actionlib_msgs.msg import GoalStatus, GoalID
from cobot_pick.msg import CobotFindObjectAction, CobotFindObjectGoal

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse

# Global Declarations
state = 0
action_client = None
goal_poses = PoseArray()
b_jntArm = False
robot_state = RobotState()

def tg_callback(msg):
  global state
  if state == 0 and msg.data is True:
    rospy.loginfo("tg_callback is True")
    state = 1

def done_cb(_state, result):
  global state
  if state is 1:
    state = 2

def feedback_cb(feedback):
  rospy.loginfo('camera feedback : ' + str(feedback))

def active_cb():
  rospy.loginfo('camera is active')

def shutdown_cb():
  global action_client
  action_client.cancel_goal()

def js_callback(msg):
  global b_jntArm, state, robot_state
  if b_jntArm is True:
    rospy.loginfo("js_callback")
    b_jntArm = False
    if state is 2:
      robot_state.joint_state = msg
      state = 3

if __name__ == "__main__":
  try:
    rospy.init_node('cobot_showcase', anonymous=True)
    sub_tg = rospy.Subscriber("/cobot/trigger", Bool, tg_callback)
    sub_tg = rospy.Subscriber("/joint_states", JointState, js_callback)

    rospy.loginfo('waiting camera server')
    action_client = actionlib.SimpleActionClient('/cobot/find_object', CobotFindObjectAction)
    action_client.wait_for_server()
    goal = CobotFindObjectGoal()
    rospy.on_shutdown(shutdown_cb)
    rospy.loginfo('camera server : ok')

    rospy.loginfo('waiting GetPositionFK server')
    rospy.wait_for_service('compute_fk')
    getFk = rospy.ServiceProxy('compute_fk', GetPositionFK)
    rospy.loginfo('GetPositionFK server : ok')

    pub_poses = rospy.Publisher("/cobot/pose", PoseArray, queue_size=10)

    global state, goal_poses, robot_state
    goal_poses.poses = [None] * 2
    old_state = -1
    while not rospy.is_shutdown():
      if old_state != state:
        rospy.loginfo("state : %d", state)
        old_state = state
        if state is 1:
          rospy.loginfo('send action camera')
          action_client.send_goal(goal=goal
            , done_cb = done_cb  # Callback that gets called on transitions to Done
            , active_cb = active_cb  # No-parameter callback that gets called on transitions to Active
            , feedback_cb = feedback_cb  # Callback that gets called whenever feedback for this goal is received
          )
        elif state is 2:
          rospy.loginfo("camera response")
          if len(action_client.get_result().poses) > 0:
            rospy.loginfo(action_client.get_result().poses)
            goal_poses.poses[1] = action_client.get_result().poses[0]
            goal_poses.poses[1].position.z = goal_poses.poses[1].position.z
            b_jntArm = True
          else:
            rospy.loginfo("target not found")
            state = 0
        elif state is 3:
          try:
            res = getFk(robot_state.joint_state.header, ['tool0'], robot_state)
            goal_poses.poses[0] = res.pose_stamped[0].pose
          except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
          rospy.loginfo(goal_poses)
          pub_poses.publish(goal_poses)
          state = 0
      rospy.sleep(0.01)
  finally:
    sub_tg.unregister()
    rospy.loginfo('end')
