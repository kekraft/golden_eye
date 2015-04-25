#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy

import sift

class Circle_Detect:

    def __init__(self, image):
        self.image = image

        self.dp = 1
        self.minDist = 20
        self.param1 = 50
        self.param2 = 30 
        self.minRadius = 0
        self.maxRadius = 0 

    def dp_update(self, level):
        self.dp = level
        self.detect_circles(self.image.copy())

    def minDist_update(self, level):
        self.minDist = level
        self.detect_circles(self.image.copy())

    def param1_update(self, level):
        self.param1 = level
        self.detect_circles(self.image.copy())

    def param2_update(self, level):
        self.param2 = level
        self.detect_circles(self.image.copy())

    def minRadius_update(self, level):
        self.minRadius = level
        self.detect_circles(self.image.copy())

    def maxRadius_update(self, level):
        self.maxRadius = level
        self.detect_circles(self.image.copy())


    def main(self, img):
        # sift.feat_detection(img)
        self.detect_circles(img)

        cv2.namedWindow('image',  cv2.WINDOW_AUTOSIZE)

        cv2.createTrackbar( "DP", "image", 1, 1000, self.dp_update)
        cv2.createTrackbar( "Min Dist", "image", 1, 50, self.minDist_update)
        cv2.createTrackbar( "Param 1", "image", 1, 100, self.param1_update)
        cv2.createTrackbar( "Param 2", "image", 1, 100, self.param2_update)
        cv2.createTrackbar( "Min Radius", "image", 0, 100, self.minRadius_update)
        cv2.createTrackbar( "Max Radius", "image", 0, 100, self.maxRadius_update)

        cv2.imshow('image', img)
        0xFF & cv2.waitKey()
        cv2.destroyAllWindows()


    def detect_circles(self, img):    
        # img = cv2.medianBlur(img, 5)
        # grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT, self.dp, self.minDist, param1=self.param1, param2=self.param2, minRadius=self.minRadius, maxRadius=self.maxRadius)
        # circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 75)
       
    #     circles = np.uint(np.around(circles, decimals=2))
        if circles is not None:
                
    #         circles = np.uint(circles, decimals=2)
            circles = np.round(circles[0, :]).astype("int")
            
            
            for (x, y, r) in circles:
                # outer circle
                cv2.circle(img, (x, y), r, (100,100,10), 4)
                # center of circle
    #             cv2.circle(grayImg, ], i[1]), 2, (0,0,255), 3)
    #          
            cv2.imshow('image', np.hstack([img]))
            # plt.imshow(np.hstack([img]))
            # plt.title("Circles")
            # plt.show()
        
        else:
            cv2.imshow('image', img) 
            # plt.imshow(img)
            # plt.title("Circles")
            # plt.show()

        # cv2.waitKey()

if __name__ == '__main__':
    # img = cv2.imread("masked.jpg", 0)
    img = cv2.imread("images/trial.cup_ring.jpg", 0)
    img = cv2.resize(img, (0,0), fx = 5, fy = 5) 
    img = cv2.GaussianBlur(img,(5,5),0)

    cd = Circle_Detect(img)
    cd.main(img)