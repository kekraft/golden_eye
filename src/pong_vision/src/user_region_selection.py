#!/usr/bin/env python 

#import freenect
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
import copy


# calibrate kinect to world
# https://github.com/amiller/libfreenect-goodies/blob/master/calibkinect.py
# also check out mouse_and_match
class User_ROI_Selection:
    NUM_FRAMES = 0
    IMAGE_ARRAY = None
    FRAMES_TO_AVERAGE = 10

    # initialize the list of reference points and boolean indicating
    # whether cropping is being performed or not
    refPt = []
    cropping = False
    image = None
    interacting = False

    def __init__(self, image):
        self.image = image


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
       
    def user_selection(self):
        # have user define and show region of interest
        # load the image, clone it, and setup the mouse callback function
        image = self.image
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
                # cv2.imwrite("saved", image)
                pass
         
               
        # close all open windows
        cv2.destroyAllWindows()
        self.interacting = False
        
     
    def main(self):
        pass
    
if __name__ == "__main__":
    img = cv2.img = cv2.imread("images/saved.jpg", 1)

    x = User_ROI_Selection(img)
    x.user_selection()