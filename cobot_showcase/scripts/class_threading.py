#!/usr/bin/env python

import sys
import os
import rospy
import threading

class SimpleThread:

    def __init__(self):
        self.t1 = threading.Thread(target=self.func1)
        self.t2 = threading.Thread(target=self.func2)
        self.t1.daemon = True
        self.t2.daemon = True
        self.t1.start()
        self.t2.start()

    def func1(self):
        while not rospy.is_shutdown():
            rospy.loginfo('func1')

    def func2(self):
        while not rospy.is_shutdown():
            rospy.loginfo('func2')

    def terminate(self):
        if self.t1.is_alive:
            self.t1.join()
        if self.t2.is_alive:
            self.t2.join()

if __name__ == "__main__":
    rospy.init_node('simple_threading', anonymous=True)
    rospy.loginfo('class_threading...')
    
    th = SimpleThread()
    rospy.on_shutdown(th.terminate)

    while not rospy.is_shutdown():
        rospy.loginfo('main')

    rospy.loginfo('end')
