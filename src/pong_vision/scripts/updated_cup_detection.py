#!/usr/bin/env python 

#import freenect
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
import copy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# calibrate kinect to world
# https://github.com/amiller/libfreenect-goodies/blob/master/calibkinect.py
# also check out mouse_and_match
class Pong_Vision:
    NUM_FRAMES = 0
    IMAGE_ARRAY = None
    FRAMES_TO_AVERAGE = 10

    # initialize the list of reference points and boolean indicating
    # whether cropping is being performed or not
    refPt = []
    cropping = False
    image = None
    interacting = False

    """
    Grabs a depth map from the Kinect sensor and creates an image from it.
    """
    def getKinectDepthMap(self):    
        depth, timestamp = freenect.sync_get_depth()
     
        np.clip(depth, 0, 2**10 - 1, depth)
        depth >>= 2
        depth = depth.astype(np.uint8)
     
        return depth

    def getKinectVideo(self):
        video, timestamp = freenect.sync_get_video()
        
        np.clip(video, 0, 2**10 - 1, video)
    #     video >>= 2
        video = video.astype(np.uint8)
        
        return video

    def displayKinect(self):
        
        
        while True:
            depth = self.getKinectDepthMap()
            video = self.getKinectVideo()
            edgesImg = self.cannyEdgeDetection(video)
            
         
    #         blur = cv2.GaussianBlur(depth, (5, 5), 0)
    #         cv2.imshow('image', blur)

            cv2.imshow('depth', depth)
            cv2.imshow('video', video)
            cv2.imshow('edges', edgesImg)
            
            self.detectCircles(video)
            
            cv2.waitKey(10)
            
            

    def cannyEdgeDetection(self, img):
        edges = cv2.Canny(img, 100, 200)
        
        # display orignal and new
    #     plt.subplot(121),plt.imshow(video,cmap = 'gray')
    #     plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    #     plt.subplot(122),plt.imshow(edgesImg,cmap = 'gray')
    #     plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    #     plt.show()    
            
        return edges

    def harris_corner_detection(self, img):
        # find Harris corners
        gray = np.float32(gray)
        dst = cv2.cornerHarris(gray,2,3,0.04)
        dst = cv2.dilate(dst,None)
        ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
        dst = np.uint8(dst)

        # find centroids
        ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

        # define the criteria to stop and refine the corners
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
        corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)

        # Now draw them
        res = np.hstack((centroids,corners))
        res = np.int0(res)
        img[res[:,1],res[:,0]]=[0,0,255]
        img[res[:,3],res[:,2]] = [0,255,0]

    def detect_circles(self, img):
        
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

    def blob_detection(self, img):

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

    def refined_blob_detection(self, img):
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
        params.minCircularity = 1
         
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

    def detect_table(self, image):
        # convert the image to grayscale, blur it, and find edges
        # in the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 11, 17, 17)
        gray = cv2.blur(gray,(3,3))
        edged = cv2.Canny(gray, 30, 200)

        cv2.imshow("Blurred Edged Scene", edged)
        cv2.waitKey(1)

        # find contours in the edged image, keep only the largest
        # ones, and initialize our screen contour
        (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
        screenCnt = None

        # loop over our contours
        # for c in cnts:
        #     # approximate the contour
        #     peri = cv2.arcLength(c, True)
        #     approx = cv2.approxPolyDP(c, 0.02 * peri, True)
     
        #     # if our approximated contour has four points, then
        #     # we can assume that we have found our screen
        #     if len(approx) == 4:
        #         screenCnt = approx
        #         break
        
        if len(cnts) > 0:
            main_cnt = cnts[0]
            perimeter = cv2.arcLength(main_cnt,True)

            epsilon = 0.2*cv2.arcLength(main_cnt,True)
            approx = cv2.approxPolyDP(main_cnt,epsilon,True)

            image_copy = copy.deepcopy(image)
            cv2.drawContours(image_copy, [approx], -1, (0, 255, 0), 3)
            cv2.imshow("Pong Table approx", image_copy)
            cv2.waitKey(1)


            cv2.drawContours(image, [main_cnt], -1, (0, 255, 0), 3)
            cv2.imshow("Pong Table Main contour", image)
            cv2.waitKey(1)

    def blurred_edge(self, image, blur_index):
        # convert the image to grayscale, blur it, and find edges
        # in the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 11, 17, 17)
        gray = cv2.blur(gray,(blur_index,blur_index))
        edged = cv2.Canny(gray, 30, 200)

        return edged
 
    def click_and_crop(self, event, x, y, flags, param):
        # print "Event, ", event
        # print "refpt, ", self.refPt
        # grab references to the global variables
        # refPt = self.refPt
        cropping = self.cropping
     
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        if event == cv2.EVENT_LBUTTONDOWN:
            print "Cropping"
            self.refPt = [(x, y)]
            cropping = True
     
        # check to see if the left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            print "Released"
            # record the ending (x, y) coordinates and indicate that
            # the cropping operation is finished
            self.refPt.append((x, y))
            cropping = False
     
            print "refpt, ", self.refPt
            # draw a rectangle around the region of interest
            if len(self.refPt) == 2:
                img_copy = self.image.copy()
                cv2.rectangle(img_copy, self.refPt[0], self.refPt[1], (155, 167, 120), 5)
                cv2.imshow("image copy", img_copy)

                 # if there are two reference points, then crop the region of interest
                # from teh image and display it
                print "Displaying cropped image"
                roi = self.image[self.refPt[0][1]:self.refPt[1][1], self.refPt[0][0]:self.refPt[1][0]]
                cv2.imshow("ROI", roi)
                cv2.waitKey(1)



            self.refPt = []
       
    def user_selection(self, image):
        # have user define and show region of interest
        # load the image, clone it, and setup the mouse callback function
        self.image = image
        clone = image.copy()
        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.click_and_crop)
         
        # keep looping until the 'q' key is pressed
        while True:
            # display the image and wait for a keypress
            cv2.imshow("image", image)
            key = cv2.waitKey(1) & 0xFF
         
            # if the 'r' key is pressed, reset the cropping region
            if key == ord("r"):
                image = clone.copy()
         
            # if the 'c' key is pressed, break from the loop
            elif key == ord("c"):
                break

            elif key == ord("s"):
                cv2.imwrite("saved", image)
         
               
        # close all open windows
        cv2.destroyAllWindows()
        self.interacting = False

    def image_cb(self, img_msg):
        height = img_msg.height
        width = img_msg.width
        # print "Width: ", width, " height: ", height

        # camera_data = img_msg.data
        # np.clip(camera_data, 0, 2**10 - 1, camera_data)
        # camera_data = camera_data.astype(np.uint8)
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

        cv2.imshow('Live Feed: Raw Image', cv_image)
        cv2.imwrite("saved.jpg", cv_image)
        cv2.waitKey(1)

        edged = self.blurred_edge(cv_image, 5)
        # cv2.imshow("Blurred Edged Scene", edged)
        # cv2.waitKey(1)

        # instantiates interactive mode for selecting regions of interest by the
        # user if we do cup selection by hand
        # if not self.interacting:
        #     print "In interacting mode"
        #     self.interacting = True
        #     self.user_selection(edged)



        # if self.IMAGE_ARRAY is not None:
        #     self.IMAGE_ARRAY = cv_image
        #     # self.IMAGE_ARRAY = self.IMAGE_ARRAY #+ cv_image
        #     # cv2.accumulate(self.IMAGE_ARRAY, cv_image)
        #     self.NUM_FRAMES += 1

        # else:
        #     self.IMAGE_ARRAY = cv_image

        # if self.NUM_FRAMES == self.FRAMES_TO_AVERAGE:
        #     print "showing averaged frame"
        #     self.NUM_FRAMES = 0
        #     cv_image = self.IMAGE_ARRAY #/ self.FRAMES_TO_AVERAGE
        #     # self.IMAGE_ARRAY = cv_image

        #     cv2.imshow('video', cv_image)
        #     # cv2.waitKey(3)

        #     # detect_circles(cv_image)
        #     # blob_detection(cv_image)
        #     self.refined_blob_detection(cv_image)
        #     self.detect_table(cv_image)
        


        
     
    def main(self):
        # displayKinect() 

        rospy.init_node('cup_detector')  

        # subscribing to laser scan so I can read it
        sub = rospy.Subscriber('/image_raw', Image, self.image_cb)

        rospy.spin()

    
if __name__ == "__main__":
#     cv2.VideoCapture.grab()
#     cv2.VideoCapture.retrieve()
#         
#     cv2.VideoCapture.read()
    x = Pong_Vision()
    x.main()