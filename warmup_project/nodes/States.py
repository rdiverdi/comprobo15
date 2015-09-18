#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, String 
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class Robot_Control():
    def __init__(self):
        rospy.init_node('estop', anonymous=True)
        rospy.Subscriber('/bump', Bump, self.bump_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.goal=90
        self.state = 'obstacle_avoid'
        self.start_time = 0.0
        self.start_time_wall = 0.0
        self.data = ()
        self.set_dist = .65


    def obstacle_avoid(self):
        things = self.find_things()
        c= 0.75
        k= 0.4

        spin = 0
        dist_sum = 0
        for angle, distance in things:
            if angle-179 < -60:
                spin -= 0.2/distance
                dist_sum += 2
            elif angle-179 < 0:
                spin += 5/distance
                dist_sum += distance-0.5
            elif angle-179 < 60:
                spin -= 5/distance
                dist_sum += distance-0.5
            elif angle-170 < 180:
                spin += 0.2/distance
                dist_sum += 2
        if len(things) == 0:
            forward = 1
            spin = 0
        else:
            forward = c*dist_sum/len(things)
            spin = k*spin/len(things)
        #print spin
        #print forward
        self.set_motion(forward=forward,spin=spin)

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

    def find_things(self):
        while self.data == ():
            pass
        if self.state == 'obstacle_avoid':
            pad = 179
            max_dist = 1.25
        elif self.state == 'follow_person':
            pad = 35
            max_dist = 1.5
        readings = []
        for i in range(2*pad):
            index = (360-pad+i)%360
            dist = self.data[index]
            readings.append(dist)
        variable_name = list(enumerate(readings))
        body_reading = []
        for reading in variable_name:
            if reading[1] > .25 and reading[1] < max_dist:
                body_reading.append(reading)
        angle_sum=0.0
        dist_sum=0.0
        for point in body_reading:
            angle_sum+=point[0]
            dist_sum+=point[1]
        if self.state == 'follow_person':
            if len(body_reading) == 0:
                if self.start_time == 0.0:
                    self.start_time = time.time()
                else:
                    if time.time() - self.start_time > 3:
                        self.state = 'obstacle_avoid'
                return (0.0, self.set_dist)
            else:
                self.start_time = 0.0
            avg_angle = angle_sum/len(body_reading)
            avg_dist = dist_sum/len(body_reading)
            return (avg_angle-35, avg_dist)
        if self.state == 'obstacle_avoid':
            return body_reading

    def follow_person(self):
        (angle, distance) = self.find_things()
        k= -1.5
        c= 1.5
        dist_diff = distance - self.set_dist
        angle_diff = -angle
        forward = c*dist_diff
        spin = k*angle_diff/45
        if dist_diff > -0.003 and dist_diff < 0.003:
            #print dist_diff
            if self.start_time_wall == 0.0:
                print 'start'
                self.start_time_wall = time.time()
            elif time.time() - self.start_time_wall > 5:
                self.state = 'find_wall'
                self.start_time_wall = 0.0
        else:
            self.start_time = 0.0

        self.set_motion(forward=forward,spin=spin)

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
            forward = abs(c/ang_err)+0.1
        if dist1 == 10 and dist2 == 10:
            if self.start_time == 0.0:
                self.start_time = time.time()
            elif time.time,() - self.start_time > 5:
                self.start_time = 0.0
                self.state = 'obstacle_avoid'
            if self.goal == 90:
                spin = 0.5
            else:
                spin = -0.5
            forward = 1
        else:
            self.start_time = 0.0
        self.set_motion(forward=forward, spin=spin)


        if self.goal == 90:
            other = self.read_angle(300)
        else:
            other = self.read_angle(60)
        if other < 0.5:
            self.state = 'obstacle_avoid'

    def read_angle(self, angle):
        pad = 5
        readings = []
        for i in range(2*pad):
            index = (angle - pad + i)%360
            dist = self.data[index]
            if dist > 0 and dist < 1.5:
                readings.append(dist)
        if readings != []:
            avg = sum(readings)/len(readings)
            return avg
        else:
            return 10

    def set_motion(self,forward=0,spin=0):
        self.forward(forward)
        self.rotate(spin)
        self.send_command()


    def forward(self,rate):
        max_speed = 0.5
        if abs(rate) < max_speed:
             self.lin_vector.x = rate
        else:
            self.lin_vector.x=rate/abs(rate)*max_speed
        #print self.lin_vector.x

    def stop(self):
        print 'stop'
        self.lin_vector.x=0.0
        self.ang_vector.z=0.0
        self.send_command()

    def rotate(self,rate):
        max_speed = 0.5
        if abs(rate) < 1:
            self.ang_vector.z=rate
        else:
            self.ang_vector.z=rate/abs(rate) *max_speed

    def wall_turn(self):
        if time.time() - self.start_time < 2:
            if self.goal == 90:
                self.rotate(-1)
                self.send_command()
            else:
                self.rotate(1)
                self.send_command()
        else:
            self.start_time = 0.0
            self.state = 'follow_wall'
              

    def send_command(self):
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

    def laser_callback(self, data):
            self.data=data.ranges

    def bump_callback(self, data):
        sensor = data.leftFront + data.leftSide + data.rightFront + data.rightSide
        if self.state == 'obstacle_avoid' and sensor > 0:
            self.state = 'follow_person'
        if self.state == 'follow_wall':
            if sensor > 0:
                self.lin_vector.x=-0.25
                self.send_command()
                self.state = 'hit_wall'
        elif self.state == 'hit_wall':
            if sensor == 0:
                self.stop()
                self.state='corner_hit'
                self.start_time = time.time()

    def main_loop(self):
            print self.state
            if self.state == 'find_wall':
                if self.data != ():
                    self.find_wall()
            if self.state == 'follow_wall':
                self.wall_align()
            if self.state == 'follow_person':
                self.follow_person()
            if self.state == 'obstacle_avoid':
                self.obstacle_avoid()
            if self.state == 'corner_hit':
                self.wall_turn()

if __name__=='__main__':
    neato = Robot_Control()
    r=rospy.Rate(10)
    while not rospy.is_shutdown():
        neato.main_loop()
        r.sleep()