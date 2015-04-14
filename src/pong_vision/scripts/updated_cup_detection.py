#!/usr/bin/env python 

#import freenect
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# calibrate kinect to world
# https://github.com/amiller/libfreenect-goodies/blob/master/calibkinect.py
# also check out mouse_and_match

"""
Grabs a depth map from the Kinect sensor and creates an image from it.
"""
def getKinectDepthMap():    
    depth, timestamp = freenect.sync_get_depth()
 
    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)
 
    return depth

def getKinectVideo():
    video, timestamp = freenect.sync_get_video()
    
    np.clip(video, 0, 2**10 - 1, video)
#     video >>= 2
    video = video.astype(np.uint8)
    
    return video

def displayKinect():
    
    
    while True:
        depth = getKinectDepthMap()
        video = getKinectVideo()
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

def detect_circles(img):
    
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

def blob_detection(img):

    # Set up the detector with default parameters.
    detector = cv2.SimpleBlobDetector()
     
    # Detect blobs.
    keypoints = detector.detect(img)
     
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
     
    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)
    cv2.waitKey(1)

def refined_blob_detection(img):
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    # Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;
     
    # Filter by Area.
    params.filterByArea = True
    # params.minArea = 1500
    params.minArea = 2000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
     
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
     
    # Show keypoints
    cv2.imshow("Refined Blob Detection", im_with_keypoints)
    cv2.waitKey(1)

def image_cb(img_msg):
    height = img_msg.height
    width = img_msg.width
    # print "Width: ", width, " height: ", height

    # camera_data = img_msg.data
    # np.clip(camera_data, 0, 2**10 - 1, camera_data)
    # camera_data = camera_data.astype(np.uint8)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    cv2.imshow('video', cv_image)
    # cv2.waitKey(3)

    detect_circles(cv_image)
    blob_detection(cv_image)
    refined_blob_detection(cv_image)

    
 
def main():
    # displayKinect() 

    rospy.init_node('cup_detector')  

    # subscribing to laser scan so I can read it
    sub = rospy.Subscriber('/image_raw', Image, image_cb)

    rospy.spin()

    
if __name__ == "__main__":
#     cv2.VideoCapture.grab()
#     cv2.VideoCapture.retrieve()
#         
#     cv2.VideoCapture.read()
    
    main()