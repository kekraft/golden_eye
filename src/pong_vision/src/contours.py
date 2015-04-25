#!/usr/bin/env python

'''
This program illustrates the use of findContours and drawContours.
The original image is put up along with the image of drawn contours.

Usage:
    contours.py
A trackbar is put up which controls the contour level from -3 to 3
'''

import numpy as np
import cv2


def mask_image(cnts, image):
    # only keep stuff from image that matches to contour
    # init mask array/image
    # mask = np.ones(image.shape[:2], dtype="uint8") * 255 # makes background white
    mask = np.ones(image.shape[:2], dtype="uint8") * 0 

    # loop over the contours
    for c in cnts:
        cv2.drawContours(mask, [c], -1, 255, -1)
        # cv2.drawContours(mask, [c], -1, 255, -1)

    # remove the contours from the image and show the resulting images
    image = cv2.bitwise_and(image, image, mask=mask)
    cv2.imshow("Mask", mask)
    cv2.imshow("After", image)
    # cv2.imwrite("masked.jpg", image)
    # cv2.imwrite("mask.jpg", image)

if __name__ == '__main__':
    print __doc__

    # img = make_image()
    # img = cv2.imread("masked.jpg", 0)
    
    img = cv2.imread("images/trial.cup_ring.jpg", 0)
    roi = img.copy()
    img = cv2.bilateralFilter(img, 11, 17, 17)
    img = cv2.blur(img,(3,3))
    img = cv2.Canny(img, 30, 200)



    h, w = img.shape[:2]

    contours0, hierarchy = cv2.findContours( img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = [cv2.approxPolyDP(cnt, 3, True) for cnt in contours0]

    def update(levels):
        vis = np.zeros((h, w, 3), np.uint8)
        levels = levels - 3
        # cv2.drawContours( vis, contours, (-1, 3)[levels <= 0], (128,255,255),
        #     3, cv2.CV_AA, hierarchy, abs(levels) )
        cv2.drawContours( vis, contours, -1, (128,255,255),
            3, cv2.CV_AA, hierarchy, abs(levels) )
        cv2.imshow('contours', vis)
        mask_image(contours, roi)

        # mask with contours
    update(3)
    cv2.createTrackbar( "levels+3", "contours", 3, 7, update )
    cv2.imshow('image', img)
    0xFF & cv2.waitKey()
    cv2.destroyAllWindows()