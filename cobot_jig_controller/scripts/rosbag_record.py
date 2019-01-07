#!/usr/bin/env python

import rospy
import subprocess
import os
import signal

class RosbagRecord:
  def __init__(self):
    self.record_script = "rosbag record -a"
#    self.record_folder = folder
    self.record_folder = "/home/dell/catkin_ws/src/cobot/record/wave_moving_6/"
    rospy.on_shutdown(self.stop_recording_handler)

    # Start recording.
    command = self.record_script
    self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
                              executable='/bin/bash')

    # Wait for shutdown signal to close rosbag record
    #rospy.spin()

  def terminate_ros_node(self, s):
    # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
      if (str.startswith(s)):
        os.system("rosnode kill " + str)

  def stop_recording_handler(self):
    rospy.loginfo(rospy.get_name() + ' stop recording.')
    self.terminate_ros_node("/record")

