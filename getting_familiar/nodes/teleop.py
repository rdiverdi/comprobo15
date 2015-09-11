#!/usr/bin/env python

'''teleoperate robot in the same manner as the exaple teleop, 
using wasd for direction and any other key press to stop'''

import tty
import select
import sys
import termios
import time
import rospy
from geometry_msgs.msg import Twist, Vector3

rospy.init_node('teleop_node')

forward_lin = Vector3(x=0.5, y=0.0, z=0.0)
forward_ang = Vector3(x=0.0, y=0.0, z=0.0)
forward_msg = Twist(linear=forward_lin, angular=forward_ang)

backward_lin = Vector3(x=-0.5, y=0.0, z=0.0)
backward_ang = Vector3(x=0.0, y=0.0, z=0.0)
backward_msg = Twist(linear=backward_lin, angular=backward_ang)

left_lin = Vector3(x=0.0, y=0.0, z=0.0)
left_ang = Vector3(x=0.0, y=0.0, z=1)
left_msg = Twist(linear=left_lin, angular=left_ang)

right_lin = Vector3(x=0.0, y=0.0, z=0.0)
right_ang = Vector3(x=0.0, y=0.0, z=-1)
right_msg = Twist(linear=right_lin, angular=right_ang)

stop_lin = Vector3(x=0.0, y=0.0, z=0.0)
stop_ang = Vector3(x=0.0, y=0.0, z=0.0)
stop_msg = Twist(linear=stop_lin, angular=stop_ang)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def forward():
	print 'forward'
	return forward_msg

def backward():
	print 'backward'
	return backward_msg

def leftturn():
	print 'left turn'
	return left_msg

def rightturn():
	print 'right turn'
	return right_msg

def stop():
	print 'stop'
	return stop_msg

def teleop():
	key = getKey()
	if key == 'w':
		pub.publish(forward())
	elif key == 's':
		pub.publish(backward())
	elif key == 'a':
		pub.publish(leftturn())
	elif key == 'd':
		pub.publish(rightturn())
	else:
		pub.publish(stop())
	time.sleep(0.1)
	return key


settings = termios.tcgetattr(sys.stdin)
key = None

r = rospy.Rate(10)
while key != '\x03':
    key = teleop()
    r.sleep()