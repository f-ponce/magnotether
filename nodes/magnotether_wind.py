#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import os
import os.path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import Queue
import time 
from std_msgs.msg import Float64

from magnotether.msg import MsgAngleData

from find_fly_angle import find_fly_angle

class ImageConverter:  

    def __init__(self):

        rospy.init_node('image_converter', anonymous=True)
        rospy.on_shutdown(self.clean_up)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)

        self.angle_pub = rospy.Publisher('/angle_data', MsgAngleData, queue_size=10)
        self.rotated_image_pub = rospy.Publisher('/rotated_image', Image, queue_size=10)

        self.contour_image_pub = rospy.Publisher('/contour_image', Image, queue_size=10)

        timestr = time.strftime("magnotether_%Y%m%d_%H%M%S", time.localtime())
        
        self.directory1 = '/home/francescavponce/catkin/src/magnotether/data'
        self.directory = '/home/francescavponce/catkin/src/magnotether/data/%s'%timestr
        #os.makedirs(self.directory)

        self.angle_filename = os.path.join(self.directory1,'angle_data_%s.csv'%timestr)
        self.angle_fid = open(self.angle_filename,'w')

        self.queue = Queue.Queue()

        self.threshold = 50
        self.mask_scale = 0.9
        self.frame_count = 0

        self.angle_list = []
        self.frame_list = []
        self.angle_data_list = []

        self.display_window = 500

        plt.ion()
        self.fig = plt.figure(1)

        self.ax = plt.subplot(1,1,1)
        self.line, = plt.plot([0,1],[0,1],'b')
        plt.grid('on')
        plt.xlabel('frame (#)')
        plt.ylabel('angle (deg)')
        plt.title('Angles vs Frame')
        self.line.set_xdata([])
        self.line.set_ydata([])
        self.ax.set_ylim(-180,180)
        self.ax.set_xlim(0,self.display_window)
        self.fig.canvas.flush_events()

        cv2.namedWindow('raw image')
        cv2.namedWindow('contour image')
        cv2.namedWindow('rotated image')

        cv2.moveWindow('raw image', 100, 100)
        cv2.moveWindow('contour image', 110, 110)
        cv2.moveWindow('rotated image', 120, 120)

    def clean_up(self):
        print('cleaning up')
        cv2.destroyAllWindows()


    def callback(self,data): 
        self.queue.put(data)


    def run(self): 

        while not rospy.is_shutdown():

            # Pull all new data from queue
            new_image_list = []

            while True:
                try:
                    ros_image = self.queue.get_nowait()
                    new_image_list.append(ros_image)
                except Queue.Empty:
                    break

            print('len(new_image_list) = {0}'.format(len(new_image_list)))


            if new_image_list:
                # Add new data to lists
                for ros_image in new_image_list:
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
                    except CvBridgeError as e:
                        print(e)

                    self.frame_count += 1
                    cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  

                    angle_rad, angle_data = find_fly_angle(cv_image_gray, self.threshold, self.mask_scale)
                    angle_deg = np.rad2deg(angle_rad)

                    self.frame_list.append(self.frame_count)
                    self.angle_list.append(angle_deg)

                    angle_data['raw_image'] = cv_image_gray
                    self.angle_data_list.append(angle_data)

                    rotated_ros_image = self.bridge.cv2_to_imgmsg(angle_data['rotated_image'])
                    rotated_ros_image.header = ros_image.header
                    
                    contour_ros_image = self.bridge.cv2_to_imgmsg(angle_data['contour_image'])
                    contour_ros_image.header = ros_image.header
                    
                    msg_angle_data = MsgAngleData()
                    msg_angle_data.header.stamp = ros_image.header.stamp
                    msg_angle_data.frame = self.frame_count
                    msg_angle_data.angle = angle_deg
                    msg_angle_data.rotated_image = rotated_ros_image

                    self.angle_pub.publish(msg_angle_data) 
                    self.rotated_image_pub.publish(rotated_ros_image)
                    self.contour_image_pub.publish(contour_ros_image)

                    self.angle_fid.write('{0} {1}\n'.format(self.frame_count, angle_deg));
                    #save image
                    #outfile = '%s.png' % (str(datetime.now()))
                    #image = cv2.imwrite(os.path.join(self.directory, outfile), cv_image)


                # Cull old data from lists
                if self.frame_list:
                    while True: 
                        if (self.frame_list[-1] - self.frame_list[0]) < self.display_window:
                            break
                        else:
                            self.frame_list.pop(0)
                            self.angle_list.pop(0)
                            self.angle_data_list.pop(0)

                    cv2.imshow("raw image", self.angle_data_list[-1]['raw_image'])
                    cv2.imshow('contour image', self.angle_data_list[-1]['contour_image'])
                    cv2.imshow('rotated image', self.angle_data_list[-1]['rotated_image'])
                    cv2.waitKey(1)
            else:
                rospy.sleep(0.001)

            # Update live plot angles
            self.line.set_xdata(self.frame_list)
            self.line.set_ydata(self.angle_list)
            if self.frame_list:
                self.ax.set_xlim(self.frame_list[0], max(self.display_window,self.frame_list[-1]))
            self.fig.canvas.flush_events()
            rospy.sleep(0.001)

        #self.angle_fid.close()


        ##saving images
#            outfile = 'thisfile.bag'#'%s.png' % (str(datetime.now()))
#            cmdline = ['rosbag', 'record','-O', outfile]
            #outfile = '%s.png' % (str(datetime.now()))
            #image = cv2.imwrite(os.path.join(self.directory, outfile), cv_image)
        #print ('saving images')



        self.angle_fid.close()

def main(args):
    ic = ImageConverter()
    ic.run()


# ---------------------------------------------------------------------------------------
if __name__ == '__main__': 
    main(sys.argv)
