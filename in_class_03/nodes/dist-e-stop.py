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

    def bump_callback(self, data):
        sensor = data.leftFront + data.leftSide + data.rightFront + data.rightSide
        if sensor > 0:
            self.lin_vector.x=-0.25
            self.send_command()
        if sensor == 0 and self.lin_vector.x<0:
            self.set_stop()
            self.send_command()

    def laser_callback(self, data):
        distance = data.ranges
        front = [dist for dist in distance[:22]]
        front.extend(distance[338:])
        filtered_dists = [dist for dist in front if dist > 0.25]
        if min(filtered_dists)<0.5:
            self.set_stop()
            self.send_command()

    def set_forward(self):
        print 'forward'
        self.lin_vector.x=0.5

    def set_stop(self):
        print 'stop'
        self.lin_vector.x=0.0
        self.ang_vector.z=0.0

    def send_command(self):
        print 'send'
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

if __name__=='__main__':
    neato = robot_control()
    neato.set_forward()
    time.sleep(1)
    neato.send_command()
    rospy.spin()
