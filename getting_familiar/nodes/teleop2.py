#!/usr/bin/env python

'''teleoperate the robot so that each button press "accelerates" 
the robot in the appropriate direction using wsad for directions 
and any other key press to stop'''

import tty
import select
import sys
import termios
import time
import rospy
from geometry_msgs.msg import Twist, Vector3

rospy.init_node('teleop_node')

lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def forward():
	print 'forward'
	if lin_vector.x<1:
		lin_vector.x+=0.1
	return Twist(linear=lin_vector, angular=ang_vector)

def backward():
	print 'backward'
	if lin_vector.x>-1:
		lin_vector.x+=-0.1
	return Twist(linear=lin_vector, angular=ang_vector)

def leftturn():
	print 'left turn'
	if ang_vector.z<1:
		ang_vector.z+=0.1
	return Twist(linear=lin_vector, angular=ang_vector)

def rightturn():
	print 'right turn'
	if ang_vector.z>-1:
		ang_vector.z+=-0.1
	return Twist(linear=lin_vector, angular=ang_vector)

def stop():
	print 'stop'
	lin_vector.x=0
	ang_vector.z=0
	return Twist(linear=lin_vector, angular=ang_vector)

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
	return key


settings = termios.tcgetattr(sys.stdin)
key = None

r = rospy.Rate(10)
while key != '\x03':
    key = teleop()
    r.sleep()