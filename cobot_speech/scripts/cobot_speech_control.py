#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from cobot_msgs.srv import *
import numpy as np

cmd_en = String()
cmd_th = String()
b_send = False
mode = String()

def cb_en(msg):
    global b_send, mode
    if "one" in msg.data or "1" in msg.data:
        print("EN : ONE")
        mode.data = 'M1'
        b_send = True
    elif "two" in msg.data or "2" in msg.data:
        print("EN : TWO")
        mode.data = 'M2'
        b_send = True
    elif "learn mode" in msg.data:
        mode.data = 'learn mode'
        b_send = True
    elif "save point" in msg.data:
        mode.data = 'save point'
        b_send = True

def cb_th(msg):
    # if not(len(cmd_th.data) is 0):
    #     cmd_th.data.pop(0)
    # cmd_th.data.append(msg)
    print(msg.data)
    print("TH : %d" % msg.data.find('หนึ่ง'))
    print("TH : %d" % msg.data.find('1'))
    if "หนึ่ง" in msg.data:
        print("TH : ONE")

if __name__ == '__main__':
    rospy.init_node('cobot_speech_control', anonymous=True)

    pub_pick_cmd = rospy.Publisher("/cobot/image2pose/cmd", String, queue_size=10);
    sub_en = rospy.Subscriber("/cobot/speech/stt/en", String, cb_en)
    sub_th = rospy.Subscriber("/cobot/speech/stt/th", String, cb_th)
    # pub = rospy.Publisher("/cobot/goal", JointState, queue_size=10)
    # print('waiting service')
    # rospy.wait_for_service('/cobot/cobot_teach/enable')
    print('start')

    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        if b_send is True:
            print('b_send = %d' % b_send)
            if "M1" in mode.data or "M2" in mode.data:
                print('aaaa')
                pub_pick_cmd.publish(mode)
            elif "learn mode" in mode.data:
                srv_enable_node = rospy.ServiceProxy('/cobot/cobot_teach/enable', EnableNode)
                res = srv_enable_node(True)
                print(res.error.data)
            elif "save point" in mode.data:
                srv_save = rospy.ServiceProxy('/cobot/cobot_core/edit_js_file', EditJointStateFile)
                res = srv_save(-1, 1)
                pub_ui = rospy.Publisher('cobot/update_ui', Bool, queue_size=10)
                pub_ui.publish(Bool(True))
                print(res.error.data)
            b_send = False
        # if cmd_en.data.find("1") != -1 or cmd_en.data.find("one") != -1 or cmd_th.data.find("หนึ่ง") != -1 or cmd_th.data.find("1") != -1:
        #     print("ONE")
        # if cmd_en.data.find("2") != -1 or cmd_en.data.find("two") != -1 or cmd_th.data.find("สอง") != -1 or cmd_th.data.find("2") != -1:
        #     print("TWO")
        # if cmd_en.data.find("pick") != -1 or cmd_th.data.find("หยิบ") != -1:
        #     print("M")
        # print(cmd_en)
        # print(cmd_th)
        # if cmd.data is 'pick box number one' or cmd.data is 'หยิบกล่องที่หนึ่ง':
        #     pub_pick_cmd.publish(String("M1"));
        # elif cmd.data is 'pick box number two' or cmd.data is 'หยิบกล่องที่สอง':
        #     pub_pick_cmd.publish(String("M2"));

        rate.sleep()

    sub_en.unregister()
    sub_th.unregister()

    pub.unregister()
    pub_status.unregister()

    print("OK !!!")
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
