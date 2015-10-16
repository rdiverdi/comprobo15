#!/usr/bin/env python

import numpy as np
import cv2
from math import *
#import rospy

cap = cv2.VideoCapture(0)
detector = cv2.FeatureDetector_create('SIFT')
extractor = cv2.DescriptorExtractor_create('SIFT')
matcher = cv2.BFMatcher()
#matcher.corner_threshold = thresh/1000.0
#matcher.ratio_threshold = ratio/100.0
first = True

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #gray = np.float32(gray)
    det = detector.detect(gray)
    dc, ext = extractor.compute(gray, det)

    if not first:
        matches = matcher.knnMatch(last_ext,ext,k=2)

        for i in range(len(det)):
                cv2.circle(frame,(int(det[i].pt[0]),int(det[i].pt[1])),2,(255,0,0),2)

        im = np.array(np.hstack((frame,lastFrame)))
        for i in range(min(len(last_det),len(det))):
                cv2.line(im,(int(det[i].pt[0]),int(det[i].pt[1])),(int(last_det[i].pt[0]+frame.shape[1]),int(last_det[i].pt[1])),(0,255,0))

        # Display the resulting frame
        cv2.imshow('frame', im)

    last_ext = ext
    last_det = det
    lastFrame = frame
    first = False
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()