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

        self.goal=90
        self.state = 'find_wall'
        self.turn_start = 0.0
        self.data=()

    def bump_callback(self, data):
        sensor = data.leftFront + data.leftSide + data.rightFront + data.rightSide
        if self.state != 'hit_wall':
            if sensor > 0:
                self.lin_vector.x=-0.25
                self.send_command()
                self.state = 'hit_wall'
        else:
            if sensor == 0:
                self.stop()
                self.state='corner_hit'


    def laser_callback(self, data):
        self.data = data.ranges

    def find_wall(self):
        dists = []
        for i in range(8):
            dists.append(self.read_angle(45*i+22))
        wall_dist = min(dists)
        angle = dists.index(wall_dist)
        if angle<4:
            self.goal=90
        else:
            self.goal=270
        self.state='follow_wall'

    def wall_align(self):
        ang_k=-1
        if self.goal == 90:
            dist_k = 0.5
        else:
            dist_k = -0.5
        c=0.03
        dist_target = 0.75
        dist1=self.read_angle(self.goal+40)
        dist2=self.read_angle(self.goal-40)
        ang_err = dist1-dist2
        avg_dist = (dist1+dist2)/2
        dist_err = avg_dist-dist_target
        spin = ang_err*ang_k+dist_err*dist_k
        if ang_err == 0:
            forward = 1
        else:
            forward = abs(c/ang_err)
        self.set_motion(forward=forward, spin=spin)

    def smart_turn(self):
        if self.goal == 90:
            self.set_motion(spin=-0.9)
        else:
            self.set_motion(spin=0.9)
        time.sleep(1.5)
        self.state='follow_wall'

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
            return 10


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
        print rate, self.goal
        if abs(rate)<1:
            self.ang_vector.z=rate
        else:
            self.ang_vector.z=rate/abs(rate)

    def send_command(self):
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

    def main_loop(self):
        print self.state
        if self.state == 'find_wall':
            if self.data != ():
                print 'wall align'
                self.find_wall()
        elif self.state == 'follow_wall':
            self.wall_align()
        elif self.state == 'corner_hit':
            self.smart_turn()



if __name__=='__main__':
    neato=robot_control()
    r=rospy.Rate(10)
    while not rospy.is_shutdown():
        neato.main_loop()
        r.sleep()
