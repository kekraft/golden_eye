#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy

import sift

class Blob_Detect:

    def __init__(self, image):
        self.image = image

        self.minThreshold = 1;
        self.maxThreshold = 2000;
         
        # Filter by Area.
        self.filterByArea = True
        self.minArea = 1
         
        # Filter by Circularity
        self.filterByCircularity = True
        self.minCircularity = 1
         
        # Filter by Convexity
        self.filterByConvexity = True
        self.minConvexity = 0.5
         
        # Filter by Inertia
        self.filterByInertia = True
        self.minInertiaRatio = 0.01



    def minThresh_update(self, level):
        self.minThreshold  = level
        self.refined_blob_detection(self.image.copy())

    def maxThresh_update(self, level):
        self.maxThreshold = level
        self.refined_blob_detection(self.image.copy())

    def minCircularity_update(self, level):
        self.minCircularity = level / 100
        self.refined_blob_detection(self.image.copy())

    def minConvexity_update(self, level):
        self.minConvexity = level / 100
        self.refined_blob_detection(self.image.copy())

    def minInertia_update(self, level):
        self.minInertiaRatio = level / 100
        self.refined_blob_detection(self.image.copy())

    def minArea_update(self, level):
        self.minArea = level
        self.refined_blob_detection(self.image.copy())


    def main(self, img):
        # sift.feat_detection(img)
        self.refined_blob_detection(img.copy())

        cv2.namedWindow('image',  cv2.WINDOW_AUTOSIZE)

        cv2.createTrackbar( "Min Threshold", "image", 1, 1000, self.minThresh_update)
        cv2.createTrackbar( "Max Threshold", "image", 1000, 2000, self.maxThresh_update)
        cv2.createTrackbar( "Min Circularity", "image", 1, 100, self.minCircularity_update)
        cv2.createTrackbar( "Min Convexity", "image", 1, 100, self.minConvexity_update)
        cv2.createTrackbar( "Min Inertia", "image", 1, 100, self.minInertia_update)
        cv2.createTrackbar( "Min Area", "image", 1, 100, self.minArea_update)

        cv2.imshow('image', img)
        0xFF & cv2.waitKey()
        cv2.destroyAllWindows()


    def refined_blob_detection(self, img):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        params.minThreshold = self.minThreshold;
        params.maxThreshold = self.maxThreshold;
         
        # Filter by Area.
        params.filterByArea = self.filterByArea
        params.minArea = self.minArea
         
        # Filter by Circularity
        params.filterByCircularity = self.filterByCircularity
        params.minCircularity = self.minCircularity
         
        # Filter by Convexity
        params.filterByConvexity = self.filterByConvexity
        params.minConvexity = self.minConvexity
         
        # Filter by Inertia
        params.filterByInertia = self.filterByInertia
        params.minInertiaRatio = self.minInertiaRatio
         
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)

         # Detect blobs.
        keypoints = detector.detect(img)
         
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
         
        # # Show keypoints
        cv2.imshow("image", im_with_keypoints)
        # # cv2.waitKey(1)
        # plt.imshow(im_with_keypoints)

if __name__ == '__main__':
    # img = cv2.imread("masked.jpg", 0)
    img = cv2.imread("images/trial.cup_ring.jpg", 0)
    # img = cv2.resize(img, (0,0), fx = 5, fy = 5) 
    img = cv2.GaussianBlur(img,(9,9),0)
    img = cv2.resize(img, (0,0), fx = 5, fy = 5) 
    # return sharp
    img = sift.sharpen(img)

    bd = Blob_Detect(img)
    bd.main(img)