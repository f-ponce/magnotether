#!/usr/bin/env python

import os, time, subprocess
import sys

import rospy

class SaveBag:
	def __init__(self,file_name,topics):
		self.topics = topics
		self.file_name = file_name

		self.filenameBag = file_name
		self.processRosbag = None

		self.time_start = time.time()
		self.max_rec_time = 3600 # if recording for more than 1 hour, just save the file

		rospy.on_shutdown(self.OnShutdown_callback)

	def OnShutdown_callback(self):
		self.StopRecordingBag()

	def StartRecordingBag(self):
		rospy.logwarn('Recording to bag file: %s' %(self.filenameBag))
		cmdline = ['rosbag','record','-O',self.filenameBag]
		cmdline.extend(self.topics)
		print cmdline
		self.processRosbag = subprocess.Popen(cmdline, preexec_fn=subprocess.os.setpgrp)

	def StopRecordingBag(self):
		subprocess.os.killpg(self.processRosbag.pid, subprocess.signal.SIGINT)
		rospy.logwarn('Closed bag file.')

	def Main(self):
		self.StartRecordingBag()
		rate = rospy.Rate(0.01)
		'''
		while not rospy.is_shutdown():
			t = time.time() - self.time_start
			if t > self.max_rec_time:
				self.StopRecordingBag()
				return
		'''

