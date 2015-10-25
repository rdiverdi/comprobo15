#!/usr/bin/env python

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Vector3, PoseWithCovariance, TwistWithCovariance, Quaternion
from std_msgs.msg import Header, ColorRGBA

import sklearn
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
        rospy.init_node('visual_odom')
        self.frame = None
        self.bridge = CvBridge()

        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        rospy.Subscriber("/odom", Odometry, self.process_odom)
        self.pub = rospy.Publisher("/Location", Marker, queue_size=10)
        
        self.tf_broadcaster = tf.TransformBroadcaster()

        #self.cap = cv2.VideoCapture(0) #get vido capture
        self.detector = cv2.FeatureDetector_create('SIFT') #setup feature detector with SIFT method
        self.extractor = cv2.DescriptorExtractor_create('SIFT') #setup extractor
        self.matcher = cv2.BFMatcher() #setup feature matcher
        self.ratio_threshold = 0.5 #thresholding for features
        self.corner_threshold = 0.02
        self.first = True #see go
        self.x = 0
        self.y = 0
        self.t = 0
        self.last_time = rospy.get_time()

        self.training_data = []
        self.train_data_in = []
        self.train_data_out = []

    def process_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def process_odom(self, msg):
        #print 'going'
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        dist = sqrt((new_x-self.x)**2+(new_y-self.y)**2)
        self.x = new_x
        self.y = new_y
        t_rate = msg.twist.twist.angular.z
        d_time = rospy.get_time()-self.last_time
        self.t = self.t+t_rate*(d_time)
        self.last_time = rospy.get_time()
        t_diff = atan2(self.y,self.x)-self.t
        if not (pi/2>t_diff and -pi/2<t_diff):
            dist = -dist
        x_rate = dist/d_time
        self.training_data = [x_rate, t_rate]
        self.build_message(self.x, self.y, self.t)

    def as_pose(self, x, y, t):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,t)
        return Pose(position=Point(x=x,y=y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

    def build_message(self, x, y, t):
        ''' build and publish an odom message given position (x,y) and angle (t) '''
        point_msg = Point(x=x, y=y, z=0.0)
        pose_msg = self.as_pose(x, y, t)

        scale_msg = Vector3(x=1, y=0.2, z=0.2)
        color_msg = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)


        header_msg = Header(stamp=rospy.Time.now(),
                            frame_id="odom")

        msg = Marker(header=header_msg, pose=pose_msg, scale=scale_msg, color=color_msg)
        msg.type = 0
        self.pub.publish(msg)

    def get_key_points(self):
        '''find matching key points between the images'''
        matches = self.matcher.knnMatch(self.last_ext, self.ext,k=2) #get matches using self.matcher
        return matches

    def key_point_filter(self, matches):
        '''filter key points to remove bad matches'''
        good_matches = []
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

    def get_movement_fundamental(self, pts_new, pts_old):
        '''get movement between images based on matched points'''
        pts_old = np.float32(pts_old) #convert to floats
        pts_new = np.float32(pts_new)
        F = cv2.findFundamentalMat(pts_old, pts_new, method=cv2.cv.CV_FM_LMEDS) #find fundamental matrix :(
        #print F[0]
        #get epipolar lines through all of the key points
        lines = cv2.computeCorrespondEpilines(pts_old.reshape(-1, 1, 2), 1, F[0])
        return lines.reshape(-1, 3) #return the lines

    def get_movement_flow(self, pts_new, pts_old):
        if len(pts_new)>0:
            dx_top_left = []
            dy_top_left = []
            dx_top_right = []
            dy_top_right = []
            dx_bottom_left = []
            dy_bottom_left = []
            dx_bottom_right = []
            dy_bottom_right = []

            for new_pt, old_pt in zip(pts_new, pts_old):
                if new_pt[1] > self.frame.shape[1]/2:
                    if new_pt[0] > self.frame.shape[1]/2:
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

            dist = [[dx_top_left, dy_top_left, 0], [dx_top_right, dy_top_right, 0],[dx_bottom_left, dy_bottom_left, 0], [dx_bottom_right, dy_bottom_right, 0]]
            for part in dist:
                part[2] = len(part[0])
                if len(part[0])>0:
                    part[0] = sum(part[0])/len(part[0])
                    #print part[0]
                    part[1] = sum(part[1])/len(part[1])
                else:
                    #print 'banana'
                    part[0]=0
                    part[1]=0
            self.train_data_in.append(np.reshape(dist, 12))
            self.train_data_out.append(self.training_data)
            return dist

    def show_lines(self, movement):
        """
        for i in range(min(len(pts_old),len(pts_new))):
            y0 = -lines[i][2]/lines[i][1]
            y1 = (-self.frame.shape[1]*lines[i][0]-lines[i][2])/lines[i][1]
            if y0 > -float('inf'):
                cv2.line(im,(0, int(y0)), (self.frame.shape[1], int(y1)), (0, 0, 255))
            
            print movement[0][0]
            print movement[1][0]
            print movement[2][0]
            print movement[3][0]
            print 'chessecake'
            cv2.arrowedLine(im, (int(pts_old[i,0]), int(pts_old[i,1])), (int(pts_new[i,0]), int(pts_new[i,1])), (0, 0, 255), thickness=3)
            cv2.circle(im,(int(pts_old[i, 0]),int(pts_old[i,1])),2,(255,255,0),2)
            cv2.circle(im,(int(pts_new[i,0]+self.frame.shape[1]),int(pts_new[i,1])),2,(255,0,0),2)
            cv2.line(im,(int(pts_old[i,0]),int(pts_old[i,1])),(int(pts_new[i,0]+self.frame.shape[1]),int(pts_new[i,1])),(0,255,0))
        """
        cv2.arrowedLine(self.frame, (int(self.frame.shape[1]*1/4), int(self.frame.shape[0]*1/4)), (int(self.frame.shape[1]*1/4+movement[0][0]), int(self.frame.shape[0]*1/4+movement[0][1])), (0, 0, 255), thickness=3)
        cv2.arrowedLine(self.frame, (int(self.frame.shape[1]*3/4), int(self.frame.shape[0]*1/4)), (int(self.frame.shape[1]*3/4+movement[1][0]), int(self.frame.shape[0]*1/4+movement[1][1])), (0, 0, 255), thickness=3)
        cv2.arrowedLine(self.frame, (int(self.frame.shape[1]*1/4), int(self.frame.shape[0]*3/4)), (int(self.frame.shape[1]*1/4+movement[2][0]), int(self.frame.shape[0]*3/4+movement[2][1])), (255, 0, 0), thickness=3)
        cv2.arrowedLine(self.frame, (int(self.frame.shape[1]*3/4), int(self.frame.shape[0]*3/4)), (int(self.frame.shape[1]*3/4+movement[3][0]), int(self.frame.shape[0]*3/4+movement[3][1])), (255, 0, 0), thickness=3)


    def go(self):
        ''' run our visual odometry implementation '''
        while self.frame == None:
            pass
        while(True): #main loop
            # Capture frame-by-frame
            #ret, self.frame = self.cap.read()

            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY) #convert to grey scale

            self.det = self.detector.detect(gray) #detect corners
            dc, self.ext = self.extractor.compute(gray, self.det) #extract corners

            #if count%10 == 0:

            if not self.first: #once there are two images
                matches = self.get_key_points() #get matched key points
                [pts_old, pts_new] = self.key_point_filter(matches) #filter out bad key points
                #lines = self.get_movement_fundamental(pts_old, pts_new)
                movement = self.get_movement_flow(pts_old, pts_new) #calculate movement and get epipolar lines
                #im = np.array(np.hstack((self.lastFrame,self.frame))) #combine images

                #plot lines on image
                self.show_lines(movement)

                # Display the resulting frame
                cv2.imshow('frame', self.frame)
            else:
                self.first = False

            #set old values to current values for next loop
            self.last_ext = self.ext
            self.last_det = self.det
            self.lastFrame = self.frame
            #time.sleep(0.25)
            if cv2.waitKey(1) & 0xFF == ord('q'): #exit on press of q
                break

        # When everything done, release the capture
        #self.cap.release()
        cv2.destroyAllWindows()
        print self.train_data_in
        print self.train_data_out

if __name__ == '__main__':
    ready_set = VisualOdometry()
    ready_set.go()
    #rospy.spin()