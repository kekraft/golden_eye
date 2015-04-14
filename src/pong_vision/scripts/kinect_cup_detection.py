#!/usr/bin/env python 

import freenect
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt

# calibrate kinect to world
# https://github.com/amiller/libfreenect-goodies/blob/master/calibkinect.py
# also check out mouse_and_match

"""
Grabs a depth map from the Kinect sensor and creates an image from it.
"""
def getDepthMap():    
    depth, timestamp = freenect.sync_get_depth()
 
    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)
 
    return depth

def getVideo():
    video, timestamp = freenect.sync_get_video()
    
    np.clip(video, 0, 2**10 - 1, video)
#     video >>= 2
    video = video.astype(np.uint8)
    
    return video

def displayKinect():
    
    
    while True:
        depth = getDepthMap()
        video = getVideo()
        edgesImg = cannyEdgeDetection(video)
        
     
#         blur = cv2.GaussianBlur(depth, (5, 5), 0)
#         cv2.imshow('image', blur)

        cv2.imshow('depth', depth)
        cv2.imshow('video', video)
        cv2.imshow('edges', edgesImg)
        
        detectCircles(video)
        
        cv2.waitKey(10)
        
        

def cannyEdgeDetection(img):
    edges = cv2.Canny(img, 100, 200)
    
    # display orignal and new
#     plt.subplot(121),plt.imshow(video,cmap = 'gray')
#     plt.title('Original Image'), plt.xticks([]), plt.yticks([])
#     plt.subplot(122),plt.imshow(edgesImg,cmap = 'gray')
#     plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
#     plt.show()    
        
    return edges

def detectCircles(img):
    
    img = cv2.medianBlur(img, 5)
    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
#     circles = cv2.HoughCircles(img, cv2.cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    circles = cv2.HoughCircles(grayImg, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 75)
   
#     circles = np.uint(np.around(circles, decimals=2))
    if circles is not None:
            
#         circles = np.uint(circles, decimals=2)
        circles = np.round(circles[0, :]).astype("int")
        
        
        for (x, y, r) in circles:
            # outer circle
            cv2.circle(img, (x, y), r, (0,255,0), 4)
            # center of circle
#             cv2.circle(grayImg, ], i[1]), 2, (0,0,255), 3)
#          
        cv2.imshow('detected circles', np.hstack([img]))
    
    else:
        cv2.imshow('detected circles', img) 
 
    
 
def main():
    displayKinect() 
    
if __name__ == "__main__":
#     cv2.VideoCapture.grab()
#     cv2.VideoCapture.retrieve()
#         
#     cv2.VideoCapture.read()
    
    main()