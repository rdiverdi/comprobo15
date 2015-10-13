#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class BlobDetector(object):
    """ The BlobDetector is a Python object that encompasses a ROS node 
        that can process images from the camera and search for blobs within """
    def __init__(self, image_topic):
        """ Initialize the blob detector """
        rospy.init_node('blob_detector')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('camera_image')             # a window for the latest camera image
        cv2.setMouseCallback('camera_image', self.process_mouse_event)
        rospy.Subscriber(image_topic, Image, self.process_image)
        cv2.namedWindow('image_info')               # a window to show color values of pixels

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

        '''
        cv2.namedWindow('threshold_image')
        self.hue_lower_bound = 0
        cv2.createTrackbar('hue lower bound', 'threshold_image', 0, 255, self.set_hue_lower_bound)
        self.hue_upper_bound = 255
        cv2.createTrackbar('hue upper bound', 'threshold_image', 0, 255, self.set_hue_upper_bound)
        self.sat_lower_bound = 0
        cv2.createTrackbar('sat lower bound', 'threshold_image', 0, 255, self.set_sat_lower_bound)
        self.sat_upper_bound = 255
        cv2.createTrackbar('sat upper bound', 'threshold_image', 0, 255, self.set_sat_upper_bound)
        self.val_lower_bound = 0
        cv2.createTrackbar('val lower bound', 'threshold_image', 0, 255, self.set_val_lower_bound)
        self.val_upper_bound = 255
        cv2.createTrackbar('val upper bound', 'threshold_image', 0, 255, self.set_val_upper_bound)

    def set_hue_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.hue_lower_bound = val
    def set_hue_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.hue_upper_bound = val
    def set_sat_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.sat_lower_bound = val
    def set_sat_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.sat_upper_bound = val
    def set_val_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.val_lower_bound = val
    def set_val_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.val_upper_bound = val
        '''

    def detect(self):
        """ Search for a blob in the last image found """
        if self.cv_image == None:
            return
        my_image = deepcopy(self.cv_image)

        for i in range(my_image.shape[0]):
            for j in range(my_image.shape[1]):
                # process pixels here
                pass

        # draw a red circle at the center of your blob
        cv2.circle(my_image, (int(0), int(0)), 5, (0,0,255),-1)
        cv2.imshow('tracking_window', my_image)
        cv2.waitKey(5)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        kernel = np.ones((11,11),'uint8')
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        #binary_image = cv2.inRange(self.hsv_image, (self.hue_lower_bound, self.sat_lower_bound, self.val_lower_bound), (self.hue_upper_bound, self.sat_upper_bound, self.val_upper_bound))
        self.binary_image = cv2.inRange(self.hsv_image, (55, 227, 11), (103, 255, 244))
        self.binary_image = cv2.erode(self.binary_image, kernel)
        cv2.imshow("camera_image", self.hsv_image)
        cv2.imshow("binary_image", self.binary_image)
        self.PID_drive()
        cv2.waitKey(5)

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.detect()
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window, 'Color (h=%d,s=%d,v=%d)' % (self.hsv_image[y,x,0], self.hsv_image[y,x,1], self.hsv_image[y,x,2]), (5,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(5)

    def get_center_of_mass(self):
        moments = cv2.moments(self.binary_image)
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
        
    def PID_drive(self):
        k = 1
        f = 1
        self.get_center_of_mass()
        height, width = self.binary_image.shape
        spin = (self.center_x - width/2)/(width/2)*k
        speed = 1-abs(spin)*f
        self.set_motion(forward=speed, spin=-spin)


    def set_motion(self, forward=0, spin=0):
        self.forward(forward)
        self.rotate(spin)
        self.send_command()


    def forward(self, speed):
        max_speed=0.5
        if abs(speed)<max_speed:
            self.lin_vector.x=speed
        else:
            self.lin_vector.x=speed/abs(speed)*max_speed

    def stop(self):
        #print 'stop'
        self.lin_vector.x=0.0
        self.ang_vector.z=0.0
        self.send_command()

    def rotate(self, rate):
        if abs(rate)<1:
            self.ang_vector.z=rate
        else:
            self.ang_vector.z=rate/abs(rate)

    def send_command(self):
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = BlobDetector("/camera/image_raw")
    node.run()