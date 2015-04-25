#!/usr/bin/env python 

import sys
sys.path.insert(0, '/opt/ros/indigo/lib/python2.7/dist-packages')

import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
import copy

## scratch work
# also check out mouse_and_match
class Surf_Test:
    
    def main(self, im_path, hessian_thres=400):
        img = cv2.imread('/home/gloin/AppliedRob/golden_eye_ws/saved.jpg',0)

        surf = cv2.SURF(hessian_thres)

        kp, des = surf.detectAndCompute(img,None)
        print 'Hessian threshold: ', len(kp)

        img2 = cv2.drawKeypoints(img,kp,None,(255,0,0),4)

        plt.imshow(img2),plt.show()

if __name__ == "__main__":

    im_path = '/home/gloin/AppliedRob/golden_eye_ws/saved.jpg'
    x = Surf_Test()
    x.main(im_path)