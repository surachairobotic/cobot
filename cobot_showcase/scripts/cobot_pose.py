#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import math
import time
import rospy
import viscid
import actionlib
import threading
import traceback
import numpy as np
from actionlib_msgs.msg import GoalStatus, GoalID
from cobot_pick.msg import CobotFindObjectAction, CobotFindObjectGoal
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R

class CobotImage:

    def __init__(self, topic):
        self.topic = topic
        self.action_client = actionlib.SimpleActionClient(topic, CobotFindObjectAction)
        self.action_client.wait_for_server()
        self.obj = CobotFindObjectGoal()

    def terminate(self):
        rospy.loginfo('CobotImage[%s] : terminate' % self.topic)
        self.isDone = True
        self.action_client.cancel_goal()

    def send_goal(self):
        self.isDone = False
        self.action_client.send_goal(goal=self.obj
            , done_cb = self.done_cb  # Callback that gets called on transitions to Done
            , active_cb = self.active_cb  # No-parameter callback that gets called on transitions to Active
            , feedback_cb = self.feedback_cb  # Callback that gets called whenever feedback for this goal is received
            )
        while self.isDone is False:
            rospy.sleep(0.01)

        res = self.action_client.get_result()
        #print("type(res) : %s" % type(res))
        #if not (res is None):
        #    if len(res.labels) > 0:
        #        for i in range(len(res.labels)):
        #            rospy.loginfo(res.labels[i])
        #            rospy.loginfo(res.poses[i])
        return res

    def done_cb(self, _state, result):
        self.isDone = True

    def feedback_cb(self, feedback):
        a=0
        #rospy.loginfo('%s feedback : %s' % (self.topic, str(feedback)))

    def active_cb(self):
        a=0
        #rospy.loginfo('%s is active' % self.topic)


class CobotImageHandler:

    def __init__(self):
        self.obj = CobotImage('/cobot/find_object')
        self.basket = CobotImage('/cobot/find_basket')
        self.newMsgs = False
        self.sub = rospy.Subscriber("/cobot/speech/text", String, self.callback)
        self.pub_lines_marker = rospy.Publisher("/cobot/marker/line_control", Marker, queue_size=10)
        self.pub = rospy.Publisher("/cobot/pose_to_control", PoseArray, queue_size=100)
        self.path = Marker()
        self.home = Pose()
        self.home.position.x = 0.25
        self.home.position.y = 0.0
        self.home.position.z = 0.264
        self.home.orientation.x = 0.999999
        self.home.orientation.y = -0.000033
        self.home.orientation.z = -0.001147
        self.home.orientation.w = -0.000007
        self.noBox = Pose()
        self.noBox.position.x = 0.200
        self.noBox.position.y = 0.250
        self.noBox.position.z = 0.065
        self.noBox.orientation = self.home.orientation
        self.noBasket = Pose()
        self.noBasket.position.x = 0.000
        self.noBasket.position.y = -0.250
        self.noBasket.position.z = 0.264
        self.noBasket.orientation = self.home.orientation
        self.seq = 0

        self.enableLoop = True
        self.enableThread = False
        self.objThread = threading.Thread(target=self.objFunc, name='objThread')
        self.basThread = threading.Thread(target=self.basFunc, name='basThread')
        self.objThread.daemon = True
        self.basThread.daemon = True
        self.objThread.start()
        self.basThread.start()
        self.enableThread = True

    def terminate(self):
        self.enableThread = False
        self.enableLoop = False
        self.obj.terminate()
        self.basket.terminate()
        if self.objThread.is_alive():
            self.objThread.join()
        if self.basThread.is_alive():
            self.basThread.join()
        self.sub.unregister()

    def objFunc(self):
        rospy.loginfo("objFunc")
        while self.enableLoop:
            if self.enableThread:
                old_t = time.time()
                self.obj_done = False
                self.obj_res = self.obj.send_goal()
                self.obj_done = True
                rospy.loginfo("object time : %lf" % (time.time()-old_t))
            rospy.sleep(0.01)
        rospy.loginfo("objFunc is done")

    def basFunc(self):
        rospy.loginfo("basFunc")
        while self.enableLoop:
            if self.enableThread:
                old_t = time.time()
                self.bas_done = False
                self.bas_res = self.basket.send_goal()
                self.bas_done = True
                rospy.loginfo("basket time : %lf" % (time.time()-old_t))
            rospy.sleep(0.01)
        rospy.loginfo("basFunc is done")

    def callback(self, msg):
        if self.newMsgs == False:
            self.input = msg.data
            rospy.loginfo('newMsgs is %s' % msg.data)
            self.newMsgs = True

    def run(self):
        rospy.logwarn('self.newMsgs is %s' % self.newMsgs)
        self.newMsgs = False
        while not rospy.is_shutdown():
            if self.newMsgs == True:
                #obj_res = self.obj.send_goal()
                self.enableThread = False
                rospy.logwarn('self.enableThread is %s' % self.enableThread)
                while not self.obj_done:
                    a=0
                _obj_res = self.obj_res
                num_obj = len(_obj_res.labels)
                if num_obj > 0:
                    b_p1 = False
                    for i in range(num_obj):
                        if _obj_res.labels[i].find(self.input) != -1:
                            p1 = _obj_res.poses[i]
                            b_p1 = True
                            rospy.loginfo("Object is found")
                            #bas_res = self.basket.send_goal()
                            while not self.bas_done:
                                a=0
                            _bas_res = self.bas_res
                            num_bas = len(_bas_res.labels)
                            if num_bas > 0:
                                b_p2 = False
                                for j in range(num_bas):
                                    if _bas_res.labels[j].find(self.input) != -1:
                                        p2 = _bas_res.poses[j]
                                        self.publish(p1, p2, 0)
                                        rospy.loginfo("Basket is found")
                                        self.say('กำลังไปหยิบกล่อง '+ self.input +' ค่ะ')
                                        b_p2 = True
                                        break
                                if b_p2 == False:
                                    p2 = self.noBasket # change home to some fix-point in workspace
                                    self.publish(p1, p2, 1)
                                    rospy.loginfo("Basket not found [B]")
                                    self.say('ไม่เจอตะกร้า '+ self.input +' ค่ะ')
                            else:
                                p2 = self.noBasket # change home to some fix-point in workspace
                                self.publish(p1, p2, 1)
                                rospy.loginfo("Basket not found [A]")
                                self.say('ไม่มีตะกร้านะคะ')
                            break
                    if b_p1 == False:
                        p1 = _obj_res.poses[0]
                        p2 = self.noBox
                        self.publish(p1, p2, 0)
                        rospy.loginfo("Object not found")
                        self.say('ไม่เจอกล่อง '+ self.input +' ค่ะ')
                else:
                    rospy.loginfo("Object is empty")
                    self.say('ไม่มีกล่องค่ะ')
                self.enableThread = True
                self.newMsgs = False
            rospy.sleep(0.05)

    def publish(self, a, b, m):
        self.path.header.frame_id = "world"
        self.path.header.stamp = rospy.Time.now()
        self.path.header.seq = self.seq
        self.path.action = self.path.ADD
        self.path.type = self.path.LINE_STRIP
        self.path.id = 0
        self.path.scale.x = 0.001
        self.path.scale.y = 0.001
        self.path.scale.z = 0.001
        self.path.color.a = 1.0
        self.path.color.r = 1.0
        self.path.color.g = 1.0
        self.path.color.b = 0.0

        poses = PoseArray()
        poses.header = Header()
        poses.header.frame_id = "world"
        poses.header.stamp = rospy.Time.now()
        poses.header.seq = self.seq
        self.path.points = []
        if m == 0:
            poses.poses = [self.p_dot(a, 0.05, 180.0), self.p_dot(a, -0.005, 180.0), self.p_dot(a, 0.05, 180.0), self.p_dot(b, 0.1, 0.0), self.p_dot(b, 0.00, 0.0), self.p_dot(b, 0.1, 0.0)]
        elif m == 1:
            poses.poses = [self.p_dot(a, 0.05, 180.0), self.p_dot(a, -0.005, 180.0), self.p_dot(a, 0.05, 180.0), self.p_dot(b, 0.00, 0.0)]

        for x in poses.poses:
            self.path.points.append(x.position)
        self.pub_lines_marker.publish(self.path)
        self.pub.publish(poses)
        self.seq = self.seq + 1

    def p_dot(self, p, d, z):
        k = z/180.0*math.pi
        rr= np.array([[math.cos(k), -math.sin(k), 0.0],
                    [ math.sin(k),  math.cos(k), 0.0],
                    [         0.0,          0.0, 1.0]])
        q = R.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        r = np.dot(q.as_dcm(), rr)

        pp = Pose()
        pp.position.x = p.position.x - r[0][2]*d
        pp.position.y = p.position.y - r[1][2]*d
        pp.position.z = p.position.z - r[2][2]*d

        qq = viscid.rotation.rot2quat(r)
        pp.orientation.w = qq[0]
        pp.orientation.x = qq[1]
        pp.orientation.y = qq[2]
        pp.orientation.z = qq[3]

        #get the real part of the quaternion first
        #pp.orientation.w = math.sqrt(float(1)+r[0][0]+r[1][1]+r[2][2])*0.5
        #pp.orientation.x = (r[2][1]-r[1][2])/(4*pp.orientation.w)
        #pp.orientation.y = (r[0][2]-r[2][0])/(4*pp.orientation.w)
        #pp.orientation.z = (r[1][0]-r[0][1])/(4*pp.orientation.w)

        #pp.orientation.x = (r[1][2]-r[2][1])/(4*pp.orientation.w)
        #pp.orientation.y = (r[2][0]-r[0][2])/(4*pp.orientation.w)
        #pp.orientation.z = (r[0][1]-r[1][0])/(4*pp.orientation.w)
        return pp

    def say(self, msgs):
        cmd = '~/catkin_ws/src/cobot/ros_speech2text/speech.sh '
        os.system(cmd+msgs)
