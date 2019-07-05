#!/usr/bin/env python

import sys
import os
import rospy

from cobot_msgs.srv import *

def test_edit_js_file():
    rospy.wait_for_service('cobot_core/edit_js_file')
    edit_js_file = rospy.ServiceProxy('cobot_core/edit_js_file', EditJointStateFile)
    rospy.loginfo("type(edit_js_file) : %s" % type(edit_js_file))
    resp = edit_js_file(3, -1)
    rospy.loginfo("type(resp) : %s" % type(resp))
    rospy.loginfo("resp : %s" % resp)

def test_read_js_file():
    rospy.wait_for_service('cobot_core/read_js_file')
    read_js_file = rospy.ServiceProxy('cobot_core/read_js_file', ReadJointStateFile)
    rospy.loginfo("type(read_js_file) : %s" % type(read_js_file))
    resp = read_js_file()
    rospy.loginfo("type(resp) : %s" % type(resp))
    rospy.loginfo("resp : %s" % resp)

if __name__ == "__main__":
    #try:
    rospy.init_node('test_core', anonymous=True)
    rospy.loginfo('test_core...')

    test_read_js_file()

    rospy.loginfo('end...')
