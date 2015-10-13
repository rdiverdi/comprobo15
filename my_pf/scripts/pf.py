#!/usr/bin/env python

""" This is the starter code for the robot localization project """

import rospy

from std_msgs.msg import Header, String, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Vector3
from nav_msgs.srv import GetMap
from copy import deepcopy
from visualization_msgs.msg import Marker, MarkerArray

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix, quaternion_from_euler
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample, randn
from sklearn.neighbors import NearestNeighbors
from occupancy_field import OccupancyField

from helper_functions import (convert_pose_inverse_transform,
                              convert_translation_rotation_to_pose,
                              convert_pose_to_xy_and_theta,
                              angle_diff)

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w          #initializes particle weight
        self.theta = theta  #initializes particle orientation
        self.x = x          #initializes particle x position
        self.y = y          #initializes particle y position

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

class ParticleFilter:
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            laser_subscriber: listens for new scan data on topic self.scan_topic
            tf_listener: listener for coordinate transforms
            tf_broadcaster: broadcaster for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
    """
    def __init__(self):
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('pf')           # tell roscore that we are creating a new node named "pf"

        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 300          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/8       # the amount of angular movement before performing an update

        self.laser_max_distance = 2.0   # maximum penalty to assess in the likelihood field model
        self.robot_pose = Pose()

        self.obstacle = rospy.Publisher('/obstacle_array', MarkerArray, queue_size=10) #initializes publisher for obstacles in front of particles

        # TODO: define additional constants if needed

        # Setup pubs and subs

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)

        # laser_subscriber listens for data from the lidar
        self.laser_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.particle_cloud = []    #initialize particle cloud

        self.current_odom_xy_theta = [0.0,0.0,0.0]  #initialize robot position

        # request the map from the map server, the map should be of type nav_msgs/OccupancyGrid

        map = self.map_reader() #loads the map using the static_map service
        
        self.occupancy_field = OccupancyField(map) #fetches the map
        self.initialized = True

        self.data = []  #initialize lidar readings

    def map_reader(self):
        """ Loads the map using the static_map service so that we can read from the map
        """
        print 'im waiting'
        rospy.wait_for_service('static_map')    #waits for the map to be available
        try:
            mapcall = rospy.ServiceProxy('static_map', GetMap)   #tries to retrieve the map once it is available
            map = mapcall().map                                  #just obtains the map section of the occupancy grid
            #print '\n\n\nim happy\n\n\n'
            return map                                          #returns the map
        except rospy.ServiceException, e:                       #if the map can't be obtained, prints the error message
            print "Service call failed: %s"%e


    def avg_angles(self, Thetas, Weights):
        """computes a weighted average of a list of angles"""
        xval = [math.cos(theta) for theta in Thetas]    # loops through thetas and generates a list of all unit circle x values
        yval = [math.sin(theta) for theta in Thetas]    #loops through thetas and generates a list of all unit circle y values
        xavg = np.ma.average(xval,weights=[Weights])    # takes a weighted average of all X values
        yavg = np.ma.average(yval,weights=[Weights])    # takes a weighted average of all Y values
        tavg = math.atan2(yavg,xavg)                     #computes arctangent of y averages and x averages
        return tavg                                     # returns the average theta value



    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles via a weighted average

        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()                          #normalizes particle weights
        X = [point.x for point in self.particle_cloud]      # loops through points and generates a list of all x values
        Y = [point.y for point in self.particle_cloud]      # loops through points and generates a list of all y values
        Theta = [point.theta for point in self.particle_cloud]  # loops through theta values and generates a list of all theta values
        Weight = [point.w for point in self.particle_cloud]     # loops through weights and generates a list of all weights
        robox = np.ma.average(X,weights=[Weight])                # takes a weighted average of all X values
        roboy = np.ma.average(Y,weights=[Weight])                # takes a weighted average of all Y values
        robot = self.avg_angles(Theta, Weight)           # takes a weighted average of all theta values

        self.robot_pose = Particle(x=robox,y=roboy,theta=robot).as_pose()   #updates robot pose with weighted averages

    def update_particles_with_odom(self, msg):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.

            msg: this is not really needed to implement this, but is here just in case.
        """
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        xscale = .05             #randomness scaling for X
        yscale = .05            #randomness scaling for Y
        tscale =  math.pi/20    #randomness scaling for rotation
        for particle in self.particle_cloud:        #loops through all particles in particle cloud
            particle.x = particle.x + delta[0]*math.cos(particle.theta-self.current_odom_xy_theta[2]) - delta[1]*math.sin(particle.theta-self.current_odom_xy_theta[2]) + xscale*randn()  #updates the X position based on the bot movement, and adds a random factor to the system
            particle.y = particle.y + delta[1]*math.cos(particle.theta-self.current_odom_xy_theta[2]) + delta[0]*math.sin(particle.theta-self.current_odom_xy_theta[2]) + yscale*randn()  #updates the Y position based on the bot movement, and adds a random factor to the system
            particle.theta += delta[2] + tscale*randn()     #updates the particle rotation based on the bot movement, and adds a random factor to the system

    def map_calc_range(self,x,y,theta):
        """ Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
        # TODO: nothing unless you want to try this alternate likelihood model
        pass

    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The particle cloud is split into a certain number of segments based on the weights of the particles. 
            The highest weighted particle section stays and new particles spawn around those ones, and regenerate the particle cloud
        """
        highest_portion = 10     #number of segments the particle cloud will be split into
        self.normalize_particles()  # make sure the distribution is normalized
        Weight = [point.w for point in self.particle_cloud]     # loops through weights and generates a list of all weights
        if len(self.particle_cloud) > 0:                           #if the particle cloud has readings
            likely_particles = self.draw_random_sample(self.particle_cloud, Weight, len(self.particle_cloud)/highest_portion)    #takes a random weighted sample from previous hypotheses that is a fraction of the original number of particles
            self.particle_cloud = []                #clears old particle cloud
            stdx = .02           #standard deviation scale of x direction
            stdy = .02          #standard deviation scale of y direction
            stdt = 0    #standard deviation scale of theta
            for particle in likely_particles:           #sweeps through the likely particles
                self.particle_cloud.append(particle)    #appends original likely particles
                for i in range(highest_portion - 1):                      #for each likely particle, appends as many particles as there were originally in particle_cloud
                    x = stdx*randn() + particle.x      #creates random x coordinate centered around the first particle with a gaussian
                    y = stdy*randn() + particle.y    #creates random y coordinate centered around the first particle with a gaussian
                    theta = stdt*randn() + particle.theta  #creates random theta centered around the first particle with a gaussian
                    self.particle_cloud.append(Particle(x,y,theta)) #appends the new particles close to the first

    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        self.data = msg.ranges    #gathers lidar readings
        sigma = .2                   # standard deviation of laser measurements
        var = sigma**2                 # variance of laser measurements
        n = 4                          # number of directions the robot is reading in, rotationally symmetric
        marker_array = MarkerArray()    #creates an array of markers
        id = 0                          #initializes marker id
        for direction in range(n):          #sweeps through laser angles
            angle = (360/n)*direction        #calculates angle laser is reading, ensures they are rotationally symmetric
            angle_rad = math.pi*angle/180   #converts angle to radians
            d = self.read_angle(angle)      #reads distance from specific angle
            for marker_number, particle in enumerate(self.particle_cloud):         #sweeps through particle hypotheses
                obstaclex = d*math.cos(particle.theta + angle_rad) + particle.x     # where obstacle x value would be if robot were at particle
                obstacley = d*math.sin(particle.theta + angle_rad) + particle.y     # where obstacle y value would be if robot were at particle
                diff = self.occupancy_field.get_closest_obstacle_distance(obstaclex,obstacley)     #determines difference between where obstacle is and where we think it is
                if diff > 0:                                            #if diff is valid, then change particle weight
                    particle.w = particle.w*(1+math.exp(-(diff**2/2*var)))   #computes gaussian distribution to weight particles based on the difference
                else:                       #doesn't update particles if the closest obstacle distance returns nan
                    print 'not updated'
                '''
                #This section adds markers where there are predicted obstacles around each particle

                pose_msg = Particle(x=obstaclex,y=obstacley,theta=particle.theta).as_pose()
                scale_msg = Vector3(x=0.1,y=0.1,z=0.1)
                color_msg = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0)
                header_msg = Header(stamp=rospy.Time.now(),frame_id='/map')
                msg = Marker(header=header_msg, pose=pose_msg, scale=scale_msg, color=color_msg)
                msg.id = id
                id += 1
                marker_array.markers.append(msg)
                msg.type=1
        self.obstacle.publish(marker_array)
        '''
        #print '\n\n\n updating \n\n\n'

    @staticmethod
    def weighted_values(values, probabilities, size):
        """ Return a random sample of size elements from the set values with the specified probabilities
            values: the values to sample from (numpy.ndarray)
            probabilities: the probability of selecting each element in values (numpy.ndarray)
            size: the number of samples
        """
        bins = np.add.accumulate(probabilities)
        return values[np.digitize(random_sample(size), bins)]

    @staticmethod
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
            choices: the values to sample from represented as a list
            probabilities: the probability of selecting each element in choices represented as a list
            n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))
        return samples

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(xy_theta)
        self.fix_map_to_odom_transform(msg)

    def initialize_particle_cloud(self, xy_theta=None):
        """ Initialize the particle cloud as a gaussian distribution around the guess
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is ommitted, the odometry will be used """
        if xy_theta == None:
            xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
<<<<<<< HEAD
        self.particle_cloud = []    #initializes particle cloud

        for i in range(300):    #creates 200 points distributed with a gaussian distribution
            stdx = .2           #standard deviation scale of x direction
            stdy = .2           #standard deviation scale of y direction
            stdt = math.pi/10    #standard deviation scale of theta
            x = stdx*randn() + xy_theta[0]      #creates random x coordinate centered around guess with a gaussian distribution
            y = stdy*randn() + xy_theta[1]      #creates random y coordinate centered around guess with a gaussian distribution
            theta = stdt*randn() + xy_theta[2]  #creates random theta centered around guess with a gaussian distribution
            self.particle_cloud.append(Particle(x,y,theta)) #appends random particle
=======
        self.particle_cloud = []
        # TODO create particles
>>>>>>> 998936e2a53a3fa1af05541b328e2ce1f6bc7c08

        self.normalize_particles()  #normalizes particles
        self.update_robot_pose()    #updates pose of robot

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        totalweight = 0.0                       #initializes sum of all particle weights
        for particle in self.particle_cloud:    # loops through particles
            totalweight += particle.w           # sums all particle weights
        for particle in self.particle_cloud:    # loops through particles
            particle.w /= totalweight           # normalizes all particle weights
            
    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=particles_conv))

    def scan_received(self, msg):
        """ This is the default logic for what to do when processing scan data.
            Feel free to modify this, however, I hope it will provide a good
            guide.  The input msg is an object of type sensor_msgs/LaserScan """
        if not(self.initialized):
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform(self.base_frame,msg.header.frame_id,msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform(self.base_frame,self.odom_frame,msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return

        # calculate pose of laser relative ot the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame,p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)

        if not(self.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud()
            # cache the last odometric pose so we can only update our particle filter if we move more than self.d_thresh or self.a_thresh
            self.current_odom_xy_theta = new_odom_xy_theta
            # update our map to odom transform now that the particles are initialized
            self.fix_map_to_odom_transform(msg)
        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
            # we have moved far enough to do an update!
            self.update_particles_with_odom(msg)    # update based on odometry
            self.update_particles_with_laser(msg)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose
            self.resample_particles()               # resample particles to focus on areas of high density
            self.fix_map_to_odom_transform(msg)     # update map to odom transform now that we have new particles
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg)

    def fix_map_to_odom_transform(self, msg):
        """ This method constantly updates the offset of the map and 
            odometry coordinate systems based on the latest results from
            the localizer """
        (translation, rotation) = convert_pose_inverse_transform(self.robot_pose)
        p = PoseStamped(pose=convert_translation_rotation_to_pose(translation,rotation),
                        header=Header(stamp=msg.header.stamp,frame_id=self.base_frame))
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        (self.translation, self.rotation) = convert_pose_inverse_transform(self.odom_to_map.pose)

    def broadcast_last_transform(self):
        """ Make sure that we are always broadcasting the last map
            to odom transformation.  This is necessary so things like
            move_base can work properly. """
        if not(hasattr(self,'translation') and hasattr(self,'rotation')):
            return
        self.tf_broadcaster.sendTransform(self.translation,
                                          self.rotation,
                                          rospy.get_rostime(),
                                          self.odom_frame,
                                          self.map_frame)

    def read_angle(self, angle):
        '''
        read the distance at an angle by averaging the readings
            from the desired angle plus or minus 5 degrees disregarding
            non-readins (avoids laser scanner glitches)
        '''
        pad = 5         #angles offset from the given that the laser will compare to
        readings = []   #initializes readings
        for i in range(2*pad):          #loops through pads around given angle
            index = (angle - pad + i)%360   #calculates actual angle to read
            dist = self.data[index]         #reads actual angle
            if dist > 0:                    #if the distance is nonzero, append the reading to readings
                readings.append(dist)
        if readings != []:                  #if readings is not empty
            avg = sum(readings)/len(readings) #computes average of all the readings
            return avg                          #returns the average
        else:
            return  10 #if there were no valid readings, return 10 (larger than any readings we saw)

if __name__ == '__main__':
    n = ParticleFilter()
    r = rospy.Rate(5)

    while not(rospy.is_shutdown()):
        # in the main loop all we do is continuously broadcast the latest map to odom transform
        n.broadcast_last_transform()
        r.sleep()