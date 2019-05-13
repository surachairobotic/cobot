#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus, GoalID
from cobot_pick.msg import CobotFindObjectAction, CobotFindObjectGoal
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseArray

class CobotImage:

  def __init__(self, topic):
    self.topic = topic
    self.action_client = actionlib.SimpleActionClient(topic, CobotFindObjectAction)
    self.action_client.wait_for_server()
    self.obj = CobotFindObjectGoal()

  def terminate(self):
    self.action_client.cancel_goal()

  def send_goal(self):
    self.isDone = False
    self.action_client.send_goal(goal=self.obj
      , done_cb = self.done_cb  # Callback that gets called on transitions to Done
      , active_cb = self.active_cb  # No-parameter callback that gets called on transitions to Active
      , feedback_cb = self.feedback_cb  # Callback that gets called whenever feedback for this goal is received
    )
    while self.isDone == False:
      rospy.sleep(0.01)
    
    res = self.action_client.get_result()
    if len(res.labels) > 0:
      for i in range(len(res.labels)):
        rospy.loginfo(res.labels[i])
        rospy.loginfo(res.poses[i])
    return res
    
  def done_cb(self, _state, result):
    rospy.loginfo('%s is done_cb' % self.topic)
    self.isDone = True

  def feedback_cb(self, feedback):
    rospy.loginfo('%s feedback : %s' % (self.topic, str(feedback)))

  def active_cb(self):
    rospy.loginfo('%s is active' % self.topic)

class CobotImageHandler:

  def __init__(self):
    self.obj = CobotImage('/cobot/find_object')
    self.basket = CobotImage('/cobot/find_basket')
    rospy.on_shutdown(self.obj.terminate)
    rospy.on_shutdown(self.basket.terminate)
    self.newMsgs = False
    sub = rospy.Subscriber("/cobot/speech/text", String, self.callback)
    self.pub = rospy.Publisher("/cobot/pose_to_control", PoseArray, queue_size=100)
    self.home = Pose()
    self.home.position.x = 0.25
    self.home.position.y = 0.0
    self.home.position.z = 0.264
    self.home.orientation.x = 0.999999
    self.home.orientation.y = -0.000033
    self.home.orientation.z = -0.001147
    self.home.orientation.w = -0.000007
    self.seq = 0

  def callback(self, msg):
    if self.newMsgs == False:
      self.input = msg.data
      self.newMsgs = True

  def run(self):
    self.newMsgs = False
    while not rospy.is_shutdown():
      if self.newMsgs == True:
        obj_res = self.obj.send_goal()
        num_obj = len(obj_res.labels)
        if num_obj > 0:
          b_p1 = False
          for i in range(num_obj):
            if obj_res.labels[i].find(self.input) != -1:
              p1 = obj_res.poses[i]
              b_p1 = True
              rospy.loginfo("Object is found");
              bas_res = self.basket.send_goal()
              num_bas = len(bas_res.labels)
              if num_bas > 0:
                b_p2 = False
                for i in range(num_bas):
                  if bas_res.labels[i].find(self.input) != -1:
                    p2 = bas_res.poses[i]
                    self.publish(p1, p2)
                    rospy.loginfo("Basket is found");
                    b_p2 = True
                    break
                if b_p1 == False:
                  p2 = self.home # change home to some fix-point in workspace
                  self.publish(p1, p2)
                  rospy.loginfo("Basket not found [B]");                  
              else:
                p2 = self.home # change home to some fix-point in workspace
                self.publish(p1, p2)
                rospy.loginfo("Basket not found [A]");              
              break
          if b_p1 == False:
            p1 = obj_res.poses[0]
            p2 = self.home # change home to some fix-point in workspace
            self.publish(p1, p2)
            rospy.loginfo("Object not found");
        else:
          rospy.loginfo("Object is empty");
      self.newMsgs = False
      rospy.sleep(0.01)

  def publish(self, a, b):
    pp = PoseArray()
    pp.header = Header()
    pp.header.seq = self.seq
    pp.header.stamp = rospy.Time.now()
    pp.header.frame_id = 'CobotImageHandler'
    pp.poses = [a, b]
    self.pub.publish(pp)
    self.seq = self.seq + 1




























