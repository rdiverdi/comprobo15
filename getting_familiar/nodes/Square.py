#!/usr/bin/env python

'''Drive the robot in a square using odometry and the framework from teleop2.py'''

import tty
import select
import sys
import termios
import time
import rospy
import tf
import math
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, String

rospy.init_node('teleop_node')

tf_listener = tf.TransformListener()
tf_br = tf.TransformBroadcaster()

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def line():
    '''drive forward 1m according to odom'''
    now = rospy.Time.now()
    tf_listener.waitForTransform("/odom", "/base_link", now+rospy.Duration(1), rospy.Duration(5.0))
    (start_trans, start_rot) = tf_listener.lookupTransform("/odom", "/base_link", now)
    pub.publish(forward())
    not_there = True
    r=rospy.Rate(10)
    while not_there:
        print 'going'
        now = rospy.Time.now()
        tf_br.sendTransform(start_trans,
                        start_rot,
                         now,
                         "start",
                         "odom")
        tf_listener.waitForTransform("/base_link", "/start", now, rospy.Duration(4.0))
        (trans,rot) = tf_listener.lookupTransform("/base_link", "/start", now)
        if math.sqrt(trans[0]**2 + trans[1]**2)>1:
            not_there = False
            return 'done'
        r.sleep()

def turn():
    '''turn 90 degrees using odom and tf as sensors'''
    now = rospy.Time.now()
    tf_listener.waitForTransform("/odom", "/base_link", now+rospy.Duration(1), rospy.Duration(4.0))
    (start_trans, start_rot) = tf_listener.lookupTransform("/odom", "/base_link", now)
    pub.publish(rightturn())
    not_there = True
    r=rospy.Rate(10)
    while not_there:
        print 'going'
        now = rospy.Time.now()
        tf_br.sendTransform(start_trans,
                        start_rot,
                         now,
                         "start",
                         "odom")
        tf_listener.waitForTransform("/base_link", "/start", now, rospy.Duration(4.0))
        (trans,rot) = tf_listener.lookupTransform("/base_link", "/start", now)
        if rot[2]>0.65:
            not_there = False
            return 'done'
        r.sleep()

def square():
    '''drive in a 1m square'''
    time.sleep(1)
    #pub.publish(forward())
    for i in range(4):
        time.sleep(0.5)
        res=line()
        print res
        pub.publish(stop())
        res = turn()
        print res
        pub.publish(stop())
    print 'yay, square'

lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

def getKey():
        '''get keypress (from example code)'''
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def forward():
    '''increase forwad speed by 0.25'''
    print 'forward'
    if lin_vector.x<1:
        lin_vector.x+=0.25
    return Twist(linear=lin_vector, angular=ang_vector)

def backward():
    '''decrease forward speed by 0.25'''
    print 'backward'
    if lin_vector.x>-1:
        lin_vector.x+=-0.25
    return Twist(linear=lin_vector, angular=ang_vector)

def leftturn():
    '''increase left turn rate by 0.25'''
    print 'left turn'
    if ang_vector.z<1:
        ang_vector.z+=0.25
    return Twist(linear=lin_vector, angular=ang_vector)

def rightturn():
    '''increase right turn rate by 0.25'''
    print 'right turn'
    if ang_vector.z>-1:
        ang_vector.z+=-0.25
    return Twist(linear=lin_vector, angular=ang_vector)

def stop():
    '''stop robot'''
    print 'stop'
    lin_vector.x=0
    ang_vector.z=0
    return Twist(linear=lin_vector, angular=ang_vector)

def teleop():
    '''teleop robot based on wasd keys'''
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

square()#drive in square
#after driving in a square, give user teleop control
r = rospy.Rate(10)
while key != '\x03':
    key=teleop()
    r.sleep()