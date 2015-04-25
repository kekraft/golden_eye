#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy

import sift

class Surf_Detector():
    def __init__(self, img):
        self.param1 = 500
        # self.param2 = 200
        self.img = img

    def surf_detect(self):
        # surf detection
        img = self.img.copy()
        surf = cv2.SURF(self.param1)
        kp, des = surf.detectAndCompute(img,None)
        img_with_kp = cv2.drawKeypoints(img,kp,None,(255,0,0),4)
        cv2.imshow("image", img_with_kp)
        # plt.imshow(img_with_kp)
        # plt.title("surf detection")
        # plt.show()

    def param1_update(self, level):
        self.param1 = level
        self.surf_detect()

    # def param2_update(self, level):
    #     self.param2 = level
    #     self.edge_detection()

    def main(self):
        # sift.feat_detection(img)       

        cv2.namedWindow('image',  cv2.WINDOW_AUTOSIZE)
        self.surf_detect()

        cv2.createTrackbar( "Param 1", "image", 500, 1000, self.param1_update)
        # cv2.createTrackbar( "Param 2", "image", 1, 2000, self.param2_update)

        cv2.imshow('image', img)
        0xFF & cv2.waitKey()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    img = cv2.imread("single.jpg", 0)
    img = cv2.resize(img, (0,0), fx = 5, fy = 5)
    img = cv2.blur(img,(3,3))
    # img = cv2.bilateralFilter(img, 11, 17, 17)
    ced = Surf_Detector(img)
    ced.main()




    