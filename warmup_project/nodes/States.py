#!/usr/bin/env python

import rospy
import time
import math
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Header, String 
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class Robot_Control():
    '''
        Control Robot in Wall Follow, Person Follow, and Obstacle Avoid modes
        Starts in obstacle avoid, 
        switches from obstacle avoid person follow on bumper press
        switches from person follow to obstacle avoid after 5 seconds of nothing in range
        switches from person follow to wall follow after 5 seconds of an object at the target distance
        switches from wall follow to obstacle avoid after 5 seconds of no wall in range
        switches from wall follow to obstacle avoid if object passes by on non-wall side
    '''
    def __init__(self):
        '''
            initialize ros node and variables which are accessed by multiple functions
        '''
        rospy.init_node('estop', anonymous=True) #initialize node
        rospy.Subscriber('/bump', Bump, self.bump_callback) #setup callback for bump sensor
        rospy.Subscriber('/scan', LaserScan, self.laser_callback) #setup callback for laser scaner

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #initialize control of robot
        
        self.lin_vector = Vector3(x=0.0, y=0.0, z=0.0) #x value = forward speed
        self.ang_vector = Vector3(x=0.0, y=0.0, z=0.0) #z value = turn rate

        self.goal=90 #wall follow target angle
        self.state = 'obstacle_avoid' #initialize state to obstacle avoid
        self.start_time = 0.0 #initialize 'start time' - used for most state switches based on timeouts
        self.start_time_wall = 0.0 #used for timeout duuring wall follow (when no wall is seen)
        self.data = () #initialize variable to store laser scan data
        self.set_dist = .65 #set distacne for wall


    def obstacle_avoid(self):
        '''
            avoid obstacles using proportional control
            groups objects into categories of front left, front right, back left, and back right
            reacts at a rate proportional to the distance to the obstacle
        '''
        things = self.find_things() #get list of angles and distances between 0.25 and 1.25 meters
        c= 0.75 #coeficient for forward speed
        k= 0.4 #coeficient for turn rate

        spin = 0
        dist_sum = 0
        for angle, distance in things:
            if angle-179 < -60:
                #pulls the robot slightly towards objects behind and to the left
                spin -= 0.2/distance #respond more to closer objects (for all cases)
                dist_sum += 2 #treat as a far away reading for speed calculation
            elif angle-179 < 0:
                #pushes robot away from objects in front and to the left
                spin += 5/distance
                dist_sum += distance-0.5
            elif angle-179 < 60:
                #pushes robot away from objects in front and to the right
                spin -= 5/distance
                dist_sum += distance-0.5
            elif angle-170 < 180:
                #pulls robot slightly towards objects behind and to the right
                spin += 0.2/distance
                dist_sum += 2
        if len(things) == 0: #avoid dividing by zero
            forward = 1
            spin = 0
        else:
            forward = c*dist_sum/len(things) #react to all objects the same way, regardless of number of readings
            spin = k*spin/len(things)
        self.set_motion(forward=forward,spin=spin) #send command to robot

    def find_wall(self):
        '''
            decide which side of the robot a wall is on
            only runs once, immediately before 'wall_align'
        '''
        dists = []
        for i in range(8):
            #get the average distance of objects in 45 degree wedges
            dists.append(self.read_angle(45*i+22))
        wall_dist = min(dists) #find wedge with the closest object
        angle = dists.index(wall_dist) #find which wedge that is
        if angle<4: #if the closest object is on the left, set target angle to 90
            self.goal=90
        else: #if the closest object is on the right, set target angle to 270
            self.goal=270
        self.state='follow_wall' #change state to 'follow wall'

    def find_things(self):
        '''Sweeps the area in .2-1.5 meters of the Neato with the lidar, and calculates the center of mass of the points it sees during find person
         -returns a tuple of the angle of the COM relative to the front of the robot and the distance of the COMfrom the robot for find person
         -returns a list of things it saw in its range during obstacle avoid'''
        while self.data == ():
            pass
        if self.state == 'obstacle_avoid':
            pad = 179 #the angle the robot sweeps to on either side, starting from the front
            max_dist = 1.25 #max of desired distacne range
        elif self.state == 'follow_person':
            pad = 35 #the angle the robot sweeps to on either side, starting from the front
            max_dist = 1.5
        readings = [] #array of lidar readings 
        for i in range(2*pad): #sweeps through the angles defined by pad
            index = (360-pad+i)%360     #calculates actual angle with respect to the robot's coordinate system
            dist = self.data[index]     #reads values from those angles
            readings.append(dist)       #adds to readings array
        variable_name = list(enumerate(readings))   #turns readings into a list of tuples containing their index and data value
        body_reading = []   #reading of points that define what the robot should follow
        for reading in variable_name:   #sweeps through readings from enumerated list
            if reading[1] > .25 and reading[1] < max_dist:    #adds all readings inside a specified box to the body_reading list
                body_reading.append(reading)
        angle_sum=0.0   #sum of all angles for COM calculation
        dist_sum=0.0    #sum of all distances for COM calculation
        for point in body_reading:  
            angle_sum+=point[0]     #sums up angles (still enumerations, but because of their nature are just offset angles)
            dist_sum+=point[1]  #sums up all distances
        if self.state == 'follow_person':
            if len(body_reading) == 0: #for first instance of no reading, intitialize start_time
                if self.start_time == 0.0:
                    self.start_time = time.time()
                else:
                    if time.time() - self.start_time > 3: #after 3 seconds of no valid readings, change state to obstacle avoid
                        self.state = 'obstacle_avoid'
                return (0.0, self.set_dist) #avoid divide by zero
            else: #if there is a valid reading, reset start_time
                self.start_time = 0.0
            avg_angle = angle_sum/len(body_reading) #get average angle
            avg_dist = dist_sum/len(body_reading) #get average distance
            return (avg_angle-35, avg_dist) #returns tuple of COM values and accounts for angle offset from enumerate
        if self.state == 'obstacle_avoid':
            return body_reading #for obstacle avoid, just return list of values in the set range

    def follow_person(self):
        '''Follows a person in front of the robot by implementing proportional control and sends commands to the robot to do so'''
        (angle, distance) = self.find_things()  #obtains angle and distance of COM
        k= -1.27   #constant defining how fast the robot will spin
        c= 1.5     #constant defining how fast the robot will move
        dist_diff = distance - self.set_dist    #the difference between the set distance to keep from the person and the actual distance
        angle_diff = -angle     #the angle difference between the person and the robot
        forward = c*dist_diff   #how fast the robot should move forward to shorten the distance
        spin = k*angle_diff/35  #how fast the robot should turn to face the person
        if dist_diff > -0.003 and dist_diff < 0.003: #if the object is at the desired distance
            if self.start_time_wall == 0.0: 
                #first time, start a timer
                print 'start'
                self.start_time_wall = time.time()
            elif time.time() - self.start_time_wall > 5:
                #after 5 seconds of the target object staying in the desired range, change state to find wall
                self.state = 'find_wall'
                self.start_time_wall = 0.0 #reset timer
        else:
            self.start_time = 0.0 #if the object moves, reset timer

        self.set_motion(forward=forward,spin=spin)

    def wall_align(self):
        '''
            follow a wall:
            read the distance to the wall at (target) plus and minus 40 degrees
            use proportional control to make the two readings equal
            also, make the distance at both angles equal to 0.75 m
        '''
        ang_k=-1 #coeficient for spin relating to the angles
        if self.goal == 90:
            dist_k = 0.5 #coeficient relating to distance with wall on the 
        else:
            dist_k = -0.5 #codficient relating to distance with wall on the right
        c=0.03 #coeficient for speed
        dist_target = 0.75 #target distance from wall
        dist1=self.read_angle(self.goal+40) #read the distance at the two angles
        dist2=self.read_angle(self.goal-40)
        ang_err = dist1-dist2
        avg_dist = (dist1+dist2)/2
        dist_err = avg_dist-dist_target
        spin = ang_err*ang_k+dist_err*dist_k #first term corrects for angle to wall, second corrects for distnace from wall
        if ang_err == 0: #avoid divide by zero
            forward = 1
        else:
            forward = abs(c/ang_err)+0.1 #control speed based on angle error (+0.1 to stay above dead zone)
        if dist1 == 10 and dist2 == 10: #if no readings in range
            if self.start_time == 0.0:
                self.start_time = time.time() #first, set start time
            elif time.time() - self.start_time > 5:
                self.start_time = 0.0
                self.state = 'obstacle_avoid' #after 5 seconds with no valid readings switch state to obstacle avoid
            if self.goal == 90: #turn towards where wall should be
                spin = 0.5
            else:
                spin = -0.5
            forward = 1
        else: #if there are valid readings, reset start_time to 0
            self.start_time = 0.0
        self.set_motion(forward=forward, spin=spin) #send command to robot

        #check 60 degrees off of forward, opposite the wall
        if self.goal == 90:
            other = self.read_angle(300)
        else:
            other = self.read_angle(60)
        if other < 0.7:
            #if there is an object within 0.7m, switch to obstacle avoid
            self.state = 'obstacle_avoid'

    def read_angle(self, angle):
        '''
            read the distance at an angle by averaging the readings
            from the desired angle plus or minus 5 degrees disregarding
            non-readins (avoids laser scanner glitches)
        '''
        pad = 5
        readings = []
        for i in range(2*pad):
            index = (angle - pad + i)%360
            dist = self.data[index]
            if dist > 0 and dist < 1.5:
                readings.append(dist)
        if readings != []:
            #normally return the average reading
            avg = sum(readings)/len(readings)
            return avg
        else:
            #if there were no valid readings, return 10 (larger than any readings we saw)
            return 10

    def set_motion(self,forward=0,spin=0):
        '''
            send desired forward and spin commands to robot
        '''
        self.forward(forward)
        self.rotate(spin)
        self.send_command()


    def forward(self,rate):
        '''
            set the robots forward speed
            limited to max_speed
        '''
        max_speed = 0.5
        if abs(rate) < max_speed:
             self.lin_vector.x = rate
        else:
            self.lin_vector.x=rate/abs(rate)*max_speed

    def stop(self):
        ''' set values to stop robot '''
        print 'stop'
        self.lin_vector.x=0.0
        self.ang_vector.z=0.0
        self.send_command()

    def rotate(self,rate):
        ''' 
        set rotation speed for the robot
        ensuring that no values over 1 are sent
        '''
        if abs(rate) < 1:
            self.ang_vector.z=rate
        else:
            self.ang_vector.z=rate/abs(rate)

    def wall_turn(self):
        '''
            turn away from wall for 2 seconds
            (used when a bump sensor indicates that a corner was hit)
        '''
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
        '''publish command to robot'''
        self.pub.publish(Twist(linear=self.lin_vector, angular=self.ang_vector))

    def laser_callback(self, data):
            '''
            set self.data to the list of distances from the laser scanner 
            when a scan is received
            '''
            self.data=data.ranges

    def bump_callback(self, data):
        '''
            callback for bump sensor
        '''
        sensor = data.leftFront + data.leftSide + data.rightFront + data.rightSide
        if self.state == 'obstacle_avoid' and sensor > 0:
            self.state = 'follow_person' #during obstacle avoid, if bumper is hit, switch to follow person
        if self.state == 'follow_wall':
            if sensor > 0: #during follow wall, if bumper is hit, start backing up and swich mode to hit wall
                self.set_motion(forward=-0.25)
                self.state = 'hit_wall'
        elif self.state == 'hit_wall':
            if sensor == 0: #once the robot is off the wall, change mode to corner hit (to turn away from the wall)
                self.stop()
                self.state='corner_hit'
                self.start_time = time.time() #set time when the turn started

    def main_loop(self):
            '''
                main run loop, runs correct function based on current state
            ''' 
            print self.state #print state
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
    neato = Robot_Control() #initialize robot
    r=rospy.Rate(10) #setup ros loop
    while not rospy.is_shutdown():
        neato.main_loop() #run main loop continuously
        r.sleep()