#!/usr/bin/env python

import sys
import os
import rospy
import math
import time
import lib_controller
import actionlib
from actionlib_msgs.msg import GoalStatus, GoalID


from affbot_kinematics.msg import AffbotMoveLastPlanAction
from affbot_kinematics.msg import AffbotMoveLastPlanGoal

b_done = False
client = None

def done_cb(state, result):
  global b_done
  print('done')
  print('state : ' + str(state))
  print('result : ' + str(result))
  b_done = True

def feedback_cb(feedback):
  print('feedback : ' + str(feedback))

def active_cb():
  print('active')


def shutdown_cb():
  global client
  client.cancel_goal()
  
if __name__ == "__main__":
  try:
    rospy.init_node('affbot_controller_client')
    client = actionlib.SimpleActionClient('/affbot/controller/move_last_plan', AffbotMoveLastPlanAction)
    client.wait_for_server()
    goal = AffbotMoveLastPlanGoal()
    print('send goal')
    client.send_goal(goal=goal
      , done_cb = done_cb  # Callback that gets called on transitions to Done
      , active_cb = active_cb  # No-parameter callback that gets called on transitions to Active
      , feedback_cb = feedback_cb  # Callback that gets called whenever feedback for this goal is received
    )
    rospy.on_shutdown(shutdown_cb)
    print('wait result')
#    client.wait_for_result()
    while not b_done:
      time.sleep(0.1)
      '''
      pb = rospy.Publisher('/affbot/controller/move_last_plan/cancel', GoalID, queue_size=100)
#        client.action_client.pub_cancel.publish(GoalID(stamp=rospy.Time(0),id=client.gh.comm_state_machine.action_goal.goal_id.id))
        
      pb.publish(GoalID(stamp=rospy.Time.now(),id=client.gh.comm_state_machine.action_goal.goal_id.id))
      if rospy.is_shutdown():
        rospy.loginfo('cancel')
        client.cancel_all_goals()
        client.gh.cancel()
        print(client.action_client.pub_cancel)
      '''
      if rospy.is_shutdown():
        break
        
        
        break
    if b_done:
      print(client.get_result())
    time.sleep(0.5)
    print('end')
    
  finally:
    pass

