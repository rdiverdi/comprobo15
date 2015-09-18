#!/usr/bin/env python

""" Experiment with receiving ROS_messages"""

import rospy
from geometry_msgs.msg import PointStamped

def process_point(msg):
	print msg

rospy.init_node('receive_message')
rospy.Subscriber('/my_point', PointStamped, process_point)

r=rospy.Rate(15)
while not rospy.is_shutdown():
	r.sleep()