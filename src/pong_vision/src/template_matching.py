#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy

import sift



if __name__ == '__main__':
    img = cv2.imread("single.jpg", 0)
    img = cv2.resize(img, (0,0), fx = 5, fy = 5) 
    img = cv2.GaussianBlur(img,(3,3),0)

    template = cv2.imread('single_ring.png',0)
    # bd = Template_Matching(img)
    # bd.main(img)

    w, h = template.shape[::-1]

    res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF_NORMED)
    threshold = 0.8
    loc = np.where( res >= threshold)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0,75,75), 2)

    cv2.imwrite('res.png',img)