#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy
import random

import sift
from blob_tunning import *

class Image_Pipeline:
    def __init__(self, verbose=True):
        self.verbose = verbose
        self.calibrated = False

        self.orig_image = None
        self.working_image = None

        # selection coordinates that worked for me...will have to 
        # tune once camera is mounted
        # [(165, 1), (246, 40)]
        self.crop_x1 = 1
        self.crop_x2 = 40
        self.crop_y1 = 165
        self.crop_y2 = 246

        self.contour_epsilon = 2

        self.canny_cup_ring_param1 = 18
        self.canny_cup_ring_param2 = 40

        self.canny_cup_blob_param1 = 30
        self.canny_cup_blob_param2 = 200


        ## ******* For Single Cup Detection **** 
        ## ******** Using a blob detector
        self.blob_minThreshold = 23;
        self.blob_maxThreshold = 357;
         
        # Filter by Area.
        self.blob_filterByArea = True
        self.blob_minArea = 6
         
        # Filter by Circularity
        self.blob_filterByCircularity = True
        self.blob_minCircularity = .23
         
        # Filter by Convexity
        self.blob_filterByConvexity = True
        self.blob_minConvexity = 0.5
         
        # Filter by Inertia
        self.blob_filterByInertia = True
        self.blob_minInertiaRatio = 0.05

        ## ******* For pixel to world function *****
        self.xy_2_world = None

    def run_pipeline(self, image):
        ''' Returns a location of a cup/cup blob '''
        self.orig_image = image
        self.working_image = image.copy()
        if self.verbose:
            print "Original Image"
            cv2.imshow("Original Image", image)
            cv2.waitKey()

        # crop the image
        self.crop_cups()

        # get cup rims
        self.get_cup_rims()

        # identify and select individual cup
        (x,y) = self.select_cup_pixel()

        # translate cup/cup rim blob to (x,y) coordinates
        world_cords = self.pixel_2_world(x,y)
        

    def crop_cups(self):
        ''' Crops the picture to center around the known cup region '''
        roi = self.working_image[self.crop_x1:self.crop_x2, self.crop_y1:self.crop_y2]
        if self.verbose:
            print "Cropped region to get cups only"
            # cv2.startWindowThread()
            cv2.imshow("Cropped ROI", roi)
            cv2.waitKey()
            cv2.destroyWindow("Cropped ROI")

            # dirty hack to make sure things finish the way they are supposed to
            cv2.waitKey(1)
            cv2.waitKey(1)
            cv2.waitKey(1)

        self.working_image = roi


    def get_cup_rims(self):
        ''' Makes the working image the cup rims with the background 
            all in black.  Assumes the cups are the largest found contour by area '''

        # filter the image....what if we did a nother blur after
        img = cv2.bilateralFilter(self.working_image, 11, 17, 17)
        img = cv2.blur(img,(3,3))
        # img = cv2.blur(img, (7,7))

        # get the canny edges
        img = cv2.Canny(img, self.canny_cup_blob_param1, self.canny_cup_blob_param2)
        if self.verbose:
            print "Canny edges found in cup rim region"
            cv2.imshow("Canny Edges in Get Cup Rims", img)
            cv2.waitKey()
            cv2.destroyWindow("Canny Edges in Get Cup Rims")

            # dirty hack to make sure things finish the way they are supposed to
            cv2.waitKey(1)
            cv2.waitKey(1)
            cv2.waitKey(1)

        # get contours of cup rims
        largest_contour = self.get_largest_contour(img)

        # mask the largest contour with the working image
        if largest_contour is None:
            print "No contour found"
        else:
            self.mask_image([largest_contour])


        

    def mask_image(self, cnts):
        ''' Masks the working image with the contour list 
        provided.

        The working image becomes the masked image
        '''
        # only keep stuff from the working image that matches to contour
        # init mask array/image
        # mask = np.ones(image.shape[:2], dtype="uint8") * 255 # makes background white
        image = self.working_image
        mask = np.ones(image.shape[:2], dtype="uint8") * 0 

        # loop over the contours
        for c in cnts:
            cv2.drawContours(mask, [c], -1, 255, -1)

        # remove the contours from the image and show the resulting images
        image = cv2.bitwise_and(image, image, mask=mask)
        
        if self.verbose:
            print "Masked image show"
            masked_copy = image.copy()
            cv2.resize(masked_copy, (0,0), fx = 25, fy = 25)
            # cv2.imshow("Mask", mask)
            cv2.imshow("Masked (Enlarged)", masked_copy)
            cv2.waitKey()
            cv2.destroyWindow("Masked (Enlarged)")
            cv2.waitKey(1)
            cv2.waitKey(1)
            cv2.waitKey(1)
            # plt.imshow(masked_copy)
            # plt.title("Mask - Enlarged")
            # plt.show()
            # cv2.imwrite("masked.jpg", image)
            # cv2.imwrite("mask.jpg", image)

        self.working_image = image

    def get_largest_contour(self, img):
        ''' Get the largest contour from the contours according to countour area '''
        contours0, hierarchy = cv2.findContours( img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # contours = [cv2.approxPolyDP(cnt, self.contour_epsilon, True) for cnt in contours0]

        cnts = sorted(contours0, key = cv2.contourArea, reverse = True)[:10]

        print "have the contours sorted"

        # mask image with the largest contour by area to only get cup rims
        if len(cnts) > 0:
            main_cnt = cnts[0]
            perimeter = cv2.arcLength(main_cnt,True)

            if self.verbose:
                print "about to display image with plt"
                image_copy = copy.deepcopy(img)
                cv2.drawContours(image_copy, [main_cnt], -1, (0, 255, 0), 3)
                # plt.imshow(image_copy)
                # plt.title("Largest contour")
                # plt.show()
                print "image displayed"

            return main_cnt

        return None

    def select_cup_pixel(self):
        # try to identify one single cup..if not get entire blob
        img = self.working_image
        img = cv2.GaussianBlur(img,(3,3),0) 
        img = cv2.Canny(img, self.canny_cup_ring_param1, self.canny_cup_ring_param2)

        if self.verbose:
            print "Canny edge dectection done"
            # plt.imshow(img)
            # plt.title("Canny Edge Detection")
            # plt.show()
            cv2.imshow("canny edge", img)
            cv2.waitKey()
            cv2.destroyWindow("canny edge")
            cv2.waitKey(1)
            cv2.waitKey(1)
            cv2.waitKey(1)
        
        # blob detection to get inside of cup
        if self.verbose:
            print "Attempting to select an individual cup"

        img = cv2.GaussianBlur(img,(9,9),0)
        img = cv2.resize(img, (0,0), fx = 5, fy = 5) 
        img = sift.sharpen(img)
        blobs = self.refined_blob_detection(img)

        if (blobs != None and blobs > 0):
            # get center of mass of cup blob or individual cup
            if self.verbose:
                for blob in blobs:
                    print "Blob centered at: ", blob.pt
            
            blob = random.choice(blobs)
            blob_cntr = blob.pt

            x = int(blob_cntr[0])
            y = int(blob_cntr[1])
        else:
            # just return center of image?? and alert user?
            print "No blob found"
            x = 0
            y = 0
        
        return x, y

    def refined_blob_detection(self, image):
        ''' Does blob detection for single cups from their rims 
        on the working image.'''
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        params.minThreshold = self.blob_minThreshold;
        params.maxThreshold = self.blob_maxThreshold;
         
        # Filter by Area.
        params.filterByArea = self.blob_filterByArea
        params.minArea = self.blob_minArea
         
        # Filter by Circularity
        params.filterByCircularity = self.blob_filterByCircularity
        params.minCircularity = self.blob_minCircularity
         
        # Filter by Convexity
        params.filterByConvexity = self.blob_filterByConvexity
        params.minConvexity = self.blob_minConvexity
         
        # Filter by Inertia
        params.filterByInertia = self.blob_filterByInertia
        params.minInertiaRatio = self.blob_minInertiaRatio
         
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)

         # Detect blobs.
        keypoints = detector.detect(image)
         
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures 
        # the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image.copy(), keypoints, np.array([]), 
            (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
         
        # # Show keypoints
        if self.verbose:
            print "Blob detection for a single cup"
            cv2.imshow("Blobs Found", im_with_keypoints)
            cv2.waitKey()
            cv2.destroyWindow("Blobs Found")
            cv2.waitKey(1)
            cv2.waitKey(1)
            cv2.waitKey(1)

        return keypoints

    def pixel_2_world(self, x,y):
        if self.verbose:
            print "Pixel Coordinates ", x, y

    def calibrate(self, image):

        ##### Calibrate the crop section
        self.orig_image = image
        calibrate_crop = Calibrate_Crop()
        calibrate_crop.start_calibration(image)

        self.crop_x1 = calibrate_crop.top_left_row
        self.crop_x2 = calibrate_crop.bot_right_row
        self.crop_y1 = calibrate_crop.top_left_col
        self.crop_y2 = calibrate_crop.bot_right_col

        self.working_image = self.orig_image.copy()

        self.crop_cups()

        ##### calibrate the blob tunning
        # blob_detector = Blob_Detect(image=None)
        # blob_detector.calibrate(self.working_image.copy())

        # # store the values
        # self.minThreshold = blob_detector.minThreshold
        # self.maxThreshold = blob_detector.maxThreshold

        # self.filterByArea = blob_detector.filterByArea
        # self.minArea = blob_detector.minArea

        # self.filterByCircularity = blob_detector.filterByCircularity
        # self.minCircularity = blob_detector.minCircularity
         
        # self.filterByConvexity = blob_detector.filterByConvexity
        # self.minConvexity = blob_detector.minConvexity
         
        # self.filterByInertia = blob_detector.filterByInertia
        # self.minInertiaRatio = blob_detector.minInertiaRatio

        ##### Cup rim calibration

    # def cup_rim_calibration():
    #     # canny detection
    #     Canny(img, self.canny_cup_blob_param1, self.canny_cup_blob_param2)


class Calibrate_Crop():
    click_ctr = 0

    def select_corners(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # print "lbutton down"

            self.click_ctr = self.click_ctr + 1

            if self.click_ctr == 1:
                self.top_left_row = y # click 1
                self.top_left_col = x

            if self.click_ctr == 2:
                self.bot_right_row = y
                self.bot_right_col = x
    

    def start_calibration(self, img = None):
        self.window = cv2.namedWindow("Select 2 Corners | Top Left, Bottom Right")
        cv2.setMouseCallback("Select 2 Corners | Top Left, Bottom Right", self.select_corners)

        assert(img != None)

        while True:
            # display the image and wait for a keypress
            # if img == None:
            #     img = cv2.imread()

            cv2.imshow("Select 2 Corners | Top Left, Bottom Right", img)
            key = cv2.waitKey(1) & 0xFF

            if self.click_ctr == 2:
                break
         
        cv2.destroyAllWindows()



if __name__ == '__main__':
    # this will probably be called from a python ROS node
    # but for now we can call it here

    # instantiate pipeline object
    ip = Image_Pipeline()

    # run pipeline with a given image 
    img = cv2.img = cv2.imread("../images/saved.jpg", 1)

    ip.run_pipeline(img)
