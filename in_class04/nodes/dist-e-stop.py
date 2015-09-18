#!/usr/bin/env python

import time
import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class robot_control(object):
    def __init__(self):
        rospy.init_node('estop', anonymous=True)
        rospy.Subscriber('/bump', Bump, self.bump_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.on_wall = False
        self.data = []

        self.P = 0.7
        self.target_dist = rospy.get_param('~target_dist')

    def bump_callback(self, data):
        sensor = data.leftFront + data.leftSide + data.rightFront + data.rightSide
        if sensor > 0:
            self.lin_vector.x=-0.25
            self.send_command()
            self.on_wall = True
        if sensor == 0 and self.on_wall:
            self.on_wall = False
            self.set_stop()
            self.send_command()

    def laser_callback(self, data):
        self.data = data.ranges
        filtered_dist = self.read_angle(0)
        speed = self.P*(filtered_dist - self.target_dist)
        self.set_forward(speed)
        print filtered_dist, speed

    def read_angle(self, angle):
        pad = 5
        readings=[]
        for i in range(2*pad):
            index = (angle-pad+i)%360
            dist = self.data[index]
            if dist > 0:
                readings.append(dist)
        if readings != []:
            return sum(readings)/len(readings)
        else:
            return 100

    def set_forward(self, speed):
        if speed<1:
            self.lin_vector.x=speed
        else:
            self.lin_vector.x=1
        self.send_command()

    def set_stop(self):
        print 'stop'
        self.lin_vector.x=0.0
        self.ang_vector.z=0.0
        self.send_command()

    def send_command(self):
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

if __name__=='__main__':
    neato = robot_control()
    time.sleep(1)
    neato.send_command()
    rospy.spin()
