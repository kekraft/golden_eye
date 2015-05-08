#!/usr/bin/env python 

import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy
import random

import sift

class Calibrate():


	def main():
		# get image from webcam for now just read it in
    	img = cv2.imread("../images/saved.jpg", 1)

		# Select crop region
		crop_region = User_ROI_Selection(img)
	    crop_region.user_selection()

	

if __name__ == '__main__':
	cal = Calibrate()
	cal.main()


