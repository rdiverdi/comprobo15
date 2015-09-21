#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, String 
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class Robot_Control():
    '''This class allows the robot to find and follow a person within 1.5 meters of the front of the robot'''
    def __init__(self):
        rospy.init_node('estop', anonymous=True)
        #rospy.Subscriber('/bump', Bump, self.bump_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0)
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
 
        self.state = 'follow_person'    #initial state of the robot
        self.data = ()  #data from lidar
        self.set_dist = .75 #distance to keep from person

    def find_person(self):
        '''Sweeps the area in .2-1.5 meters of the Neato with the lidar, and calculates the center of mass of the points it sees 
        returns a tuple of the angle of the COM relative to the front of the robot and the distance of the COMfrom the robot'''
        while self.data == ():
            pass
        pad = 35  #the angle the robot sweeps to on either side, starting from the front
        readings = []   #array of lidar readings 
        for i in range(2*pad):      #sweeps through angles defined by pad
            index = (360-pad+i)%360     #calculates actual angle with respect to the robot's coordinate system
            dist = self.data[index]     #reads values from those angles
            readings.append(dist)       #adds to readings array
        variable_name = list(enumerate(readings))   #turns readings into a list of tuples containing their index and data value
        body_reading = []   #reading of points that define what the robot should follow
        for reading in variable_name:   #sweeps through readings from enumerated list
            if reading[1] > .2 and reading[1] < 1.5:    #adds all readings inside a specified box to the body_reading list
                body_reading.append(reading)
        angle_sum=0.0   #sum of all angles for COM calculation
        dist_sum=0.0    #sum of all distances for COM calculation
        for point in body_reading:  
            angle_sum+=point[0]     #sums up angles (still enumerations, but because of their nature are just offset angles)
            dist_sum+=point[1]  #sums up all distances
        if len(body_reading) == 0:  #deals with case in which there is no body to see
            return (0.0, self.set_dist) #returns tuple of angle and specified distance to keep from person. This makes the robot will stop because of follow_person
        avg_angle = angle_sum/len(body_reading) #averages all angles
        avg_dist = dist_sum/len(body_reading)   #averages all distances
        return (avg_angle-45, avg_dist) #returns tuple of COM values and accounts for angle offset from enumerate

    def follow_person(self):
        '''Follows a person in front of the robot by implementing proportional control and sends commands to the robot to do so'''
        (angle, distance) = self.find_person()  #obtains angle and distance of COM
        k= -1.27   #constant defining how fast the robot will spin
        c= 1.5     #constant defining how fast the robot will move
        dist_diff = distance - self.set_dist    #the difference between the set distance to keep from the person and the actual distance
        angle_diff = -angle     #the angle difference between the person and the robot
        forward = c*dist_diff   #how fast the robot should move forward to shorten the distance
        spin = k*angle_diff/35  #how fast the robot should turn to face the person
        # print dist_diff
        self.set_motion(forward=forward,spin=spin)  #sends command to robot

    def set_motion(self,forward=0,spin=0):
        '''sends commands to the robot so that it will move'''
        self.forward(forward)   #determines velocity
        self.rotate(spin)   #determines rotation
        self.send_command()     #publishes commands to robot


    def forward(self,rate):
        '''determines the linear velocity of the robot'''
        max_speed = 0.5     #maximum speed of the robot
        if abs(rate) < max_speed:   #if the rate is less than the maximum speed, define fowrad velocity as the rate
             self.lin_vector.x = rate
        else:
            self.lin_vector.x=rate/abs(rate)*max_speed  #if the rate is more than the max speed, send the robot at the max speed in the direction of rate
        #print self.lin_vector.x

    def stop(self):
        '''stops the robot'''
        #print 'stop'
        self.lin_vector.x=0.0   #sets the linear velocity to 0
        self.ang_vector.z=0.0   #sets the angular velocity to 0
        self.send_command()     #publishes command to robot

    def rotate(self,rate):
        '''determines the spin of the robot'''
        max_speed = 1     #maximum turning velocity of the robot
        if abs(rate) < max_speed:   #if the turning speed is less than the max speed, define the spin speed as rate
            self.ang_vector.z=rate
        else:
            self.ang_vector.z=rate/abs(rate) *max_speed     #if the turning speed is more than the max speed, spin the robot at top speed in direction of rate
              

    def send_command(self):
        '''publishes commands to the robot so that it can move'''
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))    #publishes the linear and angular vectors

    def laser_callback(self, data):
        '''reads data from the laser'''
        self.data=data.ranges   #sets the data to self.data

    def main_loop(self):
        '''runs the entire follow person sequence of the robot'''
            #print self.state
        if self.state == 'follow_person':   #if the state of the robot is follow_person then run the follow_person functions
            self.follow_person()

if __name__=='__main__':
    neato = Robot_Control()     #defines neato as robot_control class
    r=rospy.Rate(10)            #timing rate for messages
    while not rospy.is_shutdown():
        neato.main_loop()   #runs the commands for the robot
        r.sleep()           #sleeps until next loop