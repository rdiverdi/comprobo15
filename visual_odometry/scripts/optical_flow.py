#!/usr/bin/env python

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Vector3, PoseWithCovariance, TwistWithCovariance, Quaternion
from std_msgs.msg import Header, ColorRGBA

import sklearn
from sklearn.linear_model import LinearRegression
import tf
import numpy as np
import cv2
from math import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import rospy

class VisualOdometry():
    '''track movement based on changing camera images'''
    def __init__(self):
        '''setup lots of variables'''
        rospy.init_node('visual_odom') #intitialze ros node
        self.frame = None #initialize camera frame variable to None
        self.bridge = CvBridge() #used to convert camera image to opencv image

        rospy.Subscriber("/camera/image_raw", Image, self.process_image) #subscribe to camera image
        rospy.Subscriber("/odom", Odometry, self.process_odom) #subscribe to odometry (for training)
        self.pub = rospy.Publisher("/Location", Marker, queue_size=10) #setup publisher to show position of robot

        self.detector = cv2.FeatureDetector_create('SIFT') #setup feature detector with SIFT method
        self.extractor = cv2.DescriptorExtractor_create('SIFT') #setup extractor
        self.matcher = cv2.BFMatcher() #setup feature matcher
        self.ratio_threshold = 0.6 #thresholding for features
        self.corner_threshold = 0.01
        self.first = True #see go
        self.x = 0 #init robot position
        self.y = 0
        self.t = 0 #robot theta
        self.last_time = rospy.get_time() #initialize 'last_time' to the current time

        self.train = True #start in training mode
        self.training_data = [] #initialize training data arrays
        self.train_data_in = []
        self.train_data_out = []
        self.model = LinearRegression() #initialize training model as a linear regression model

    def process_image(self, msg):
        ''' do all initial processing to the image '''
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_odom(self, msg):
        ''' 
            in training mode, save incoming odometry data as training data
            also publish a marker which should mimic odom
        '''
        if self.train:
            t_const = 1 #incoming angle rate may or may not be in rad/s
            new_x = msg.pose.pose.position.x #get x from odom
            new_y = msg.pose.pose.position.y #get y from odom
            x_diff = new_x - self.x #get deltas
            y_diff = new_y - self.y
            dist = sqrt(x_diff**2.+y_diff**2) #calculate distance traveled
            self.x = new_x #reset old x and y
            self.y = new_y
            t_rate = msg.twist.twist.angular.z*t_const #get angle change rate from odom
            d_time = rospy.get_time()-self.last_time #get change in time
            self.t = self.t+t_rate*(d_time) #calculate current angle from t_rate and d_time
            self.last_time = rospy.get_time() #reset last_time
            t_drive = atan2(y_diff,x_diff) #find angle of travel
            t_diff = self.t - t_drive #compare travel angle to robot angle
            if not (pi/2>abs(t_diff)): #if they are in approximately opposite directions
                print 'hello'
                dist = -dist #set direction of travel to backwards
            x_rate = dist/d_time #calculate forward travel rate
            self.training_data = [x_rate, t_rate]
            self.build_message(self.x, self.y, self.t) #make marker to show odom position

    def as_pose(self, x, y, t):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,t)
        return Pose(position=Point(x=x,y=y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

    def build_message(self, x, y, t):
        ''' build and publish a marker message given position (x,y) and angle (t) '''
        pose_msg = self.as_pose(x, y, t) #build a pose message with the input position and angle

        scale_msg = Vector3(x=1, y=0.2, z=0.2) #set size of arrow
        color_msg = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0) #set color of arrow

        header_msg = Header(stamp=rospy.Time.now(), #set header with current time
                            frame_id="odom") #use odom frame as reference

        #combine above variables into the final message
        msg = Marker(header=header_msg, pose=pose_msg, scale=scale_msg, color=color_msg)
        msg.type = 0 #set the marker to be an arrow
        self.pub.publish(msg) #publish hte marker

    def get_key_points(self):
        '''find matching key points between the images'''
        matches = self.matcher.knnMatch(self.last_ext, self.ext,k=2) #get matches using self.matcher
        return matches

    def key_point_filter(self, matches):
        '''filter key points to remove bad matches'''
        good_matches = [] #init list of matches
        for m,n in matches:
            # make sure the distance to the closest match is sufficiently better than the second closest
            if (m.distance < self.ratio_threshold*n.distance and
                self.last_det[m.queryIdx].response > self.corner_threshold and
                self.det[m.trainIdx].response > self.corner_threshold):
                good_matches.append((m.queryIdx, m.trainIdx))
            
        pts_old = np.zeros((len(good_matches),2)) #init list of points
        pts_new = np.zeros((len(good_matches),2))


        for idx in range(len(good_matches)): #create list of old an new points (separated by image)
            match = good_matches[idx]
            pts_old[idx,:] = self.last_det[match[0]].pt
            pts_new[idx,:] = self.det[match[1]].pt
        return [pts_old, pts_new]

    def get_movement_flow(self, pts_new, pts_old):
        ''' get robot movement using an optical flow technique '''
        if len(pts_new)>0: #only run if points are input
            #initialize a list for x and y values in each of the 4 quadrants
            dx_top_left = []
            dy_top_left = []
            dx_top_right = []
            dy_top_right = []
            dx_bottom_left = []
            dy_bottom_left = []
            dx_bottom_right = []
            dy_bottom_right = []

            for new_pt, old_pt in zip(pts_new, pts_old): #cycle through points
                #find which quadrant the point is in
                if new_pt[1] > self.frame.shape[1]/2:
                    if new_pt[0] > self.frame.shape[1]/2:
                        #append the point's change in position to the correct lists
                        dx_bottom_right.append(new_pt[0] - old_pt[0])
                        dy_bottom_right.append(new_pt[1] - old_pt[1])
                    else:
                        dx_bottom_left.append(new_pt[0] - old_pt[0])
                        dy_bottom_left.append(new_pt[1] - old_pt[1])
                else:
                    if new_pt[0] > self.frame.shape[1]/2:
                        dx_top_right.append(new_pt[0] - old_pt[0])
                        dy_top_right.append(new_pt[1] - old_pt[1])
                    else:
                        dx_top_left.append(new_pt[0] - old_pt[0])
                        dy_top_left.append(new_pt[1] - old_pt[1])

            #combine into a single array
            dist = [[dx_top_left, dy_top_left, 0], [dx_top_right, dy_top_right, 0],[dx_bottom_left, dy_bottom_left, 0], [dx_bottom_right, dy_bottom_right, 0]]
            for part in dist: #for each quadrant
                part[2] = len(part[0]) #store the number of points in the quadrant
                if len(part[0])>0: #if there are pints
                    part[0] = sum(part[0])/len(part[0]) #get the average dx
                    part[1] = sum(part[1])/len(part[1]) #get the average dy
                else: #if there aren't any points, output [0,0] for that quadrant
                    part[0]=0
                    part[1]=0

            #during train mode, append the calculated averages and the most recent odom data to the training sets
            if self.train:
                self.train_data_in.append(np.reshape(dist, 12))
                self.train_data_out.append(self.training_data)

            #when not training, predict the motion using the model
            else:
                d_time = rospy.get_time()-self.last_time #get delta time
                out = self.model.predict(np.reshape(dist, 12)) #predict the motion
                df = out[0,0]*d_time #separate into forward motion
                dt = out[0,1]*d_time #and rotation
                self.t = self.t + dt #add the motion to the current position and angle
                self.x = self.x + df*cos(self.t) #move in the direction of the current angle
                self.y = self.y + df*sin(self.t)
                self.build_message(self.x, self.y, self.t) #output the position as a marker
                self.last_time = rospy.get_time() #reset time
            return dist #return the average matrix (used for plotting)

    def show_lines(self, movement):
        ''' 
        Plots the average vectors of each of the four quadrants of the image 
        '''
        #quadrant 1 vector
        cv2.arrowedLine(self.frame, (int(self.frame.shape[1]*1/4), int(self.frame.shape[0]*1/4)), (int(self.frame.shape[1]*1/4+movement[0][0]), int(self.frame.shape[0]*1/4+movement[0][1])), (0, 0, 255), thickness=3)
        #quadrant 2 vector
        cv2.arrowedLine(self.frame, (int(self.frame.shape[1]*3/4), int(self.frame.shape[0]*1/4)), (int(self.frame.shape[1]*3/4+movement[1][0]), int(self.frame.shape[0]*1/4+movement[1][1])), (0, 0, 255), thickness=3)
        #quadrant 3 vector
        cv2.arrowedLine(self.frame, (int(self.frame.shape[1]*1/4), int(self.frame.shape[0]*3/4)), (int(self.frame.shape[1]*1/4+movement[2][0]), int(self.frame.shape[0]*3/4+movement[2][1])), (255, 0, 0), thickness=3)
        #quadrant 4 vector
        cv2.arrowedLine(self.frame, (int(self.frame.shape[1]*3/4), int(self.frame.shape[0]*3/4)), (int(self.frame.shape[1]*3/4+movement[3][0]), int(self.frame.shape[0]*3/4+movement[3][1])), (255, 0, 0), thickness=3)

    def go(self):
        ''' run our visual odometry implementation '''
        while self.frame == None: #does nothing if there is no previous frame
            pass
        self.last_time = rospy.get_time()  #initializing a time 
        while(True): #main loop
            # Capture frame-by-frame
            #ret, self.frame = self.cap.read()
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY) #convert to grey scale

            self.det = self.detector.detect(gray) #detect corners
            dc, self.ext = self.extractor.compute(gray, self.det) #extract corners

            if not self.first: #once there are two images start matching keypoints
                matches = self.get_key_points() #get matched key points
                [pts_old, pts_new] = self.key_point_filter(matches) #filter out bad key points
                movement = self.get_movement_flow(pts_old, pts_new) #calculate movement and get epipolar lines
                
                self.show_lines(movement) #plot vectors on image

                cv2.imshow('frame', self.frame) # Display the resulting frame
            else:
                self.first = False #after the first image, it isn't the first image any more

            #set old values to current values for next loop
            self.last_ext = self.ext
            self.last_det = self.det
            self.lastFrame = self.frame
            #End the run loop when 'q' is pressed to move on to the "trained" mode or quit
            if cv2.waitKey(1) & 0xFF == ord('q'): #exit on press of q
                break

        # When everything done, close everything
        cv2.destroyAllWindows()

    def train_odom(self):
        '''
        Computes the matrix required to transform between the image vectors and the real odometry using a linear regression
        '''
        self.model.fit(self.train_data_in, self.train_data_out) #computes the matrix to multiply by the train_data_in (from the camera) to get the odometry
        self.train = False #the Neato's training is complete. It is now a master of Visual Odometry

if __name__ == '__main__':
    ready_set = VisualOdometry() #initialize class
    ready_set.go()  #runs the code and collects data
    ready_set.train_odom() #computes the matrix to find the odometry
    ready_set.go() #actually does visual odometry