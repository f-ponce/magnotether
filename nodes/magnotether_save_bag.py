#!/usr/bin/env python
from __future__ import division
import rospy
from optparse import OptionParser
import tf
import sys
import time, os, subprocess
import threading
import numpy as np
import rosbag
from sensor_msgs.msg import Image
# import cv: open cv 1 not used
from cv_bridge import CvBridge, CvBridgeError

import imp

#
class SaveBag:
  def __init__(self):
    directory = '/home/francescavponce/catkin/src/magnotether/data'
    self.record_length_seconds = 300
    self.time_start = time.time()
    experiment_basename = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    filename = experiment_basename + '_' + '.bag'
    self.filenameBag = os.path.expanduser(os.path.join(directory, filename))
    self.processRosbag = None
    #rospy.on_shutdown(self.OnShutdown_callback)

  def OnShutdown_callback(self):
    self.StopRecordingBag()
        
  def StartRecordingBag(self, filename):
    rospy.logwarn('Saving bag file: %s' % (self.filenameBag))
    cmdline = ['rosbag', 'record','-O', self.filenameBag]
    cmdline.extend(self.topics)
    print cmdline
    self.processRosbag = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)
    
#  def StopRecordingBag(self):
#    subprocess.os.killpg(self.processRosbag.pid, subprocess.signal.SIGINT)
#    rospy.logwarn('Closed bag file.')
                
  def Main(self):
    savebag.StartRecordingBag()
    rate = rospy.Rate(0.01)
    while not rospy.is_shutdown():
        t = time.time() - self.time_start
        if t > self.record_length_seconds:
            self.StopRecordingBag()      
            return
        

if __name__ == '__main__':

    rospy.init_node('SaveBag', log_level=rospy.INFO)
    rospy.sleep(1)
    savebag = SaveBag()
    savebag.Main()
     
