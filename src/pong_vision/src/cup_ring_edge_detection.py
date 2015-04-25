#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy

import sift

class Canny_Edge_Detector():
    def __init__(self, img):
        self.param1 = 100
        self.param2 = 200
        self.img = img

    def edge_detection(self):
        img = self.img
        edges = cv2.Canny(img, self.param1, self.param2)
        cv2.imshow("image", edges)

    def param1_update(self, level):
        self.param1 = level
        self.edge_detection()

    def param2_update(self, level):
        self.param2 = level
        self.edge_detection()

    def main(self):
        # sift.feat_detection(img)       

        cv2.namedWindow('image',  cv2.WINDOW_AUTOSIZE)
        self.edge_detection()

        cv2.createTrackbar( "Param 1", "image", 1, 1000, self.param1_update)
        cv2.createTrackbar( "Param 2", "image", 1, 2000, self.param2_update)

        cv2.imshow('image', img)
        0xFF & cv2.waitKey()
        cv2.destroyAllWindows()



if __name__ == '__main__':


    img = cv2.imread("images/single.jpg", 0)
    img = cv2.resize(img, (0,0), fx = 5, fy = 5) 
    img = cv2.blur(img,(3,3))
    # img = cv2.bilateralFilter(img, 11, 17, 17)
    ced = Canny_Edge_Detector(img)
    ced.main()

    # img = cv2.resize(img, (0,0), fx = 5, fy = 5) 
    
    # img = cv2.blur(img,(3,3))
    # edges = cv2.GaussianBlur(img,(5,5),0)

    # for x in xrange(20):
    #     img = sift.sharpen(img)

    # sift.cannyEdgeDetection(img)
    # # display orignal and new
    # plt.subplot(121),plt.imshow(img,cmap = 'gray')
    # plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(122),plt.imshow(edges,cmap = 'gray')
    # plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    # plt.show()    