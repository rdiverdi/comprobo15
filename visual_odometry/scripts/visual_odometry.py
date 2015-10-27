#!/usr/bin/env python

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

        #self.cap = cv2.VideoCapture(0) #get vido capture
        self.detector = cv2.FeatureDetector_create('SIFT') #setup feature detector with SIFT method
        self.extractor = cv2.DescriptorExtractor_create('SIFT') #setup extractor
        self.matcher = cv2.BFMatcher() #setup feature matcher
        self.ratio_threshold = 0.6 #thresholding for features
        self.corner_threshold = 0.01
        self.first = True #see go

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

    def process_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

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
                lines = self.get_movement_fundamental(pts_old, pts_new) #calculate movement and get epipolar lines

                im = np.array(np.hstack((self.lastFrame,self.frame))) #combine images

                #plot key points, connecting lines, and epipolar lines
                for i in range(min(len(pts_old),len(pts_new))):
                        y0 = -lines[i][2]/lines[i][1]
                        y1 = (-self.frame.shape[1]*lines[i][0]-lines[i][2])/lines[i][1]
                        if y0 > -float('inf'):
                            cv2.line(im,(0, int(y0)), (self.frame.shape[1], int(y1)), (0, 0, 255))
                        cv2.circle(im,(int(pts_old[i, 0]),int(pts_old[i,1])),2,(255,255,0),2)
                        cv2.circle(im,(int(pts_new[i,0]+self.frame.shape[1]),int(pts_new[i,1])),2,(255,0,0),2)
                        #cv2.line(im,(int(pts_old[i,0]),int(pts_old[i,1])),(int(pts_new[i,0]+self.frame.shape[1]),int(pts_new[i,1])),(0,255,0))

                # Display the resulting frame
                cv2.imshow('frame', im)
            else:
                self.first = False

            #set old values to current values for next loop
            self.last_ext = self.ext
            self.last_det = self.det
            self.lastFrame = self.frame
            time.sleep(0.5)
            if cv2.waitKey(1) & 0xFF == ord('q'): #exit on press of q
                break

        # When everything done, release the capture
        #self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    ready_set = VisualOdometry()
    ready_set.go()