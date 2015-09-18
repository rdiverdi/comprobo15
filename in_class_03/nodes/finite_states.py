#!/usr/bin/env python

import time
import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class robot_control():
    def __init__(self):
        rospy.init_node('estop', anonymous=True)
        rospy.Subscriber('/bump', Bump, self.bump_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.state = 'forward'
        self.turn_start = 0.0

    def bump_callback(self, data):
        if self.state == 'forward':
            sensor = data.leftFront + data.leftSide + data.rightFront + data.rightSide
            if sensor > 0:
                self.lin_vector.x=-0.25
                self.send_command()
                self.state = 'backward'
        if self.state == 'rotate left':
            self.rotate()

    def laser_callback(self, data):
        if self.state == 'backward':
            distance = data.ranges
            front = [dist for dist in distance[:22]]
            front.extend(distance[338:])
            filtered_dists = [dist for dist in front if dist > 0.25]
            if min(filtered_dists)>0.5:
                self.set_rotate_left()
                self.send_command()
                self.state = 'rotate left'
                self.turn_start = rospy.Time.now()

    def set_forward(self):
        print 'forward'
        self.lin_vector.x=0.5
        self.ang_vector.z=0.0

    def set_stop(self):
        print 'stop'
        self.lin_vector.x=0.0
        self.ang_vector.z=0.0

    def set_rotate_left(self):
        print 'spin left'
        self.lin_vector.x=0.0
        self.ang_vector.z=1

    def send_command(self):
        print 'send'
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

    def rotate(self):
        if rospy.Time.now() > (self.turn_start + rospy.Duration(1.0)):
            self.set_forward()
            self.send_command()
            self.state = 'forward'

if __name__=='__main__':
    neato = robot_control()
    neato.set_forward()
    time.sleep(1)
    neato.send_command()
    rospy.spin()
