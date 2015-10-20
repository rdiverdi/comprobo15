#!/usr/bin/env python

import numpy as np
import cv2
from math import *
import time
#import rospy

class VisualOdometry():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.detector = cv2.FeatureDetector_create('SIFT')
        self.extractor = cv2.DescriptorExtractor_create('SIFT')
        self.matcher = cv2.BFMatcher()
        #matcher.corner_threshold = thresh/1000.0
        #matcher.ratio_threshold = ratio/100.0
        self.ratio_threshold = 5
        self.corner_threshold = 0.02
        self.first = True

    def get_key_points(self):
        matches = self.matcher.knnMatch(self.last_ext, self.ext,k=2)

        #for i in range(len(self.det)):
        #        cv2.circle(self.frame,(int(self.det[i].pt[0]),int(self.det[i].pt[1])),2,(255,0,0),2)
        return matches

    def key_point_filter(self, matches):
        good_matches = []
        for m,n in matches:
            # make sure the distance to the closest match is sufficiently better than the second closest
            if (m.distance < self.ratio_threshold*n.distance and
                self.last_det[m.queryIdx].response > self.corner_threshold and
                self.det[m.trainIdx].response > self.corner_threshold):
                good_matches.append((m.queryIdx, m.trainIdx))
            
        pts_old = np.zeros((len(good_matches),2))
        pts_new = np.zeros((len(good_matches),2))


        for idx in range(len(good_matches)):
            match = good_matches[idx]
            pts_old[idx,:] = self.last_det[match[0]].pt
            pts_new[idx,:] = self.det[match[1]].pt
        return [pts_old, pts_new]

    def get_movement(self, pts_new, pts_old):
        pts_old = np.float32(pts_old)
        pts_new = np.float32(pts_new)
        F = cv2.findFundamentalMat(pts_old, pts_new, method=cv2.cv.CV_FM_LMEDS)
        print F[0]
        lines = cv2.computeCorrespondEpilines(pts_old.reshape(-1, 1, 2), 1, F[0])
        return lines.reshape(-1, 3)


    def go(self):
        while(True):
            # Capture frame-by-frame
            ret, self.frame = self.cap.read()

            # Our operations on the frame come here
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

            #gray = np.float32(gray)
            self.det = self.detector.detect(gray)
            dc, self.ext = self.extractor.compute(gray, self.det)

            if not self.first:
                matches = self.get_key_points()
                [pts_old, pts_new] = self.key_point_filter(matches)
                lines = self.get_movement(pts_old, pts_new)

                im = np.array(np.hstack((self.lastFrame,self.frame)))

                for i in range(min(len(pts_old),len(pts_new))):
                        y0 = -lines[i][2]/lines[i][1]
                        y1 = (-self.frame.shape[1]*lines[i][0]-lines[i][2])/lines[i][1]
                        if y0 > -float('inf'):
                            cv2.line(im,(0, int(y0)), (self.frame.shape[1], int(y1)), (0, 0, 255))
                        cv2.circle(im,(int(pts_old[i, 0]),int(pts_old[i,1])),2,(255,255,0),2)
                        cv2.circle(im,(int(pts_new[i,0]+self.frame.shape[1]),int(pts_new[i,1])),2,(255,0,0),2)
                        cv2.line(im,(int(pts_old[i,0]),int(pts_old[i,1])),(int(pts_new[i,0]+self.frame.shape[1]),int(pts_new[i,1])),(0,255,0))

                # Display the resulting frame
                cv2.imshow('frame', im)

            self.last_ext = self.ext
            self.last_det = self.det
            self.lastFrame = self.frame
            self.first = False
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    ready_set = VisualOdometry()
    ready_set.go()