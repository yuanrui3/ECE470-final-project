# -*- coding: utf-8 -*-
"""
Created on Sat Nov 13 18:30:04 2021

@author: 14677
"""

import numpy as np
import cv2
from geometry_msgs.msg import Point

# Params for camera calibration
theta = 0
beta = 1
tx = 0
ty = 0

def IMG2W(x,y):
    #T = np.array([1,0,0,tx],[0,1,0,ty],[0,0,1,0],[0,0,0,1])
    yw=((x-394)/beta)*1000+ty
    xw=((y-256)/beta)*1000+tx
    return [xw,yw]

def blob_search(image_raw, color):
    # Setup SimpleBlobDetector parameters.
     params = cv2.SimpleBlobDetector_Params()
     # Filter by Color
     params.filterByColor = False
     # Filter by Area.
     params.filterByArea = True
     params.maxArea = 800
     params.minArea = 250
     # Filter by Circularity
     params.filterByCircularity = False
     params.maxCircularity = 0.82
     params.minCircularity = 0.7
     # Filter by Inerita
     params.filterByInertia = False
     params.maxInertiaRatio = 1
     params.minInertiaRatio = 0.8
     # Filter by Convexity
     params.filterByConvexity = False
     # Create a detector with the parameters
     detector = cv2.SimpleBlobDetector_create(params)
     # Convert the image into the HSV color space
     hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
     if (color=='yellow'):
         lower = (20,150,50) # yellow lower
         upper = (36,255,255) # yellow upper
     elif (color=='brown') :
         lower = (35,150,50) # brown lower
         upper = (74,255,255) # brown upper
     else :
         lower = (35,150,50) # green lower
         upper = (74,255,255) # green upper
     
     # Define a mask using the lower and upper bounds of the target color
     mask_image = cv2.inRange(hsv_image, lower, upper)
     keypoints = detector.detect(mask_image)
     # Find blob centers in the image coordinates
     blob_image_center = []
     #num_blobs = len(keypoints)
     num_blobs = len(keypoints)
     for i in range(num_blobs):
         blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
     # Draw the keypoints on the detected block
     im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]),(0,0,255))
     xw_yw = []
     if(num_blobs == 0):
         pass
         print("No block found!")
     else:
     # Convert image coordinates to global world coordinate using IM2W() function
         for i in range(num_blobs):
             xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
     #cv2.namedWindow("Camera View")
     #cv2.imshow("Camera View", image_raw)
     #cv2.namedWindow("Mask View")
     #cv2.imshow("Mask View", mask_image)
     cv2.namedWindow("Keypoint View")
     cv2.imshow("Keypoint View", im_with_keypoints)
     cv2.waitKey(2)
     return xw_yw
