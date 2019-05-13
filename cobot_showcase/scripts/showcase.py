#!/usr/bin/env python

import sys
import os
import rospy

import time
import actionlib
from geometry_msgs.msg import Pose, PoseArray
from actionlib_msgs.msg import GoalStatus, GoalID
from cobot_pick.msg import CobotFindObjectAction, CobotFindObjectGoal

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, DisplayTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse

# Global Declarations
state = 0
action_client_object = None
action_client_basket = None
goal_poses = PoseArray()
b_jntArm = False
robot_state = RobotState()
box = String()

def speech_callback(msg):
  global state, box
  if state == 0:
    rospy.loginfo("speech_callback is True")
    box = msg
    state = 1

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
#  if state is 1:
#    state = 2

def feedback_cb_basket(feedback):
  rospy.loginfo('camera basket feedback : ' + str(feedback))

def active_cb_basket():
  rospy.loginfo('camera basket is active')

def shutdown_cb():
  global action_client_object, action_client_basket
  action_client_object.cancel_goal()
  action_client_basket.cancel_goal()

def js_callback(msg):
  global b_jntArm, state, robot_state
  if b_jntArm is True:
    rospy.loginfo("js_callback")
    b_jntArm = False
    if state is 2:
      robot_state.joint_state = msg
      state = 3

def tj_callback(msg):
  global state
  if state is 4:
    state = 5

if __name__ == "__main__":
  try:
    rospy.init_node('cobot_showcase', anonymous=True)
    rospy.loginfo('cobot_showcase . . .')
    sub_tg = rospy.Subscriber("/cobot/speech/text", String, speech_callback)
    sub_tg = rospy.Subscriber("/joint_states", JointState, js_callback)
    sub_tj = rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, tj_callback)
    pub_exe = rospy.Publisher("/cobot/execute", Bool, queue_size=10)

    global state, goal_poses, robot_state, action_client_object, action_client_basket

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

    pub_poses = rospy.Publisher("/cobot/pose", PoseArray, queue_size=10)

    goal_poses.poses = [None] * 2
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
              if action_client_object.get_result().labels[i].find(box.data) != -1:
                goal_poses.poses[1] = action_client_object.get_result().poses[i]
                goal_poses.poses[1].position.z = goal_poses.poses[1].position.z + 0.01
                b_jntArm = True
                rospy.loginfo(goal_poses.poses[1])
                break
          if not b_jntArm:
            rospy.loginfo("box not found")
            state = 0
        elif state is 3:
          try:
            res = getFk(robot_state.joint_state.header, ['tool0'], robot_state)
            goal_poses.poses[0] = res.pose_stamped[0].pose
            #goal_poses.poses[1].orientation = goal_poses.poses[0].orientation
          except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
          rospy.loginfo(goal_poses)
          pub_poses.publish(goal_poses)
          state = 4
        elif state is 5:
          s = Bool()
          s.data = True
          pub_exe.publish(s)
          state = 0
      rospy.sleep(0.01)
  finally:
    sub_tg.unregister()
    rospy.loginfo('end')
