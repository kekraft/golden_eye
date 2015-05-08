#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
import copy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class Pong_System:
	def __init__(self):
		self.motor_a = Motor("motor_a")
		self.motor_b = Motor("motor_b")
		self.motor_c = Motor("motor_c")

		self.game_state_sub
		self.launcher_sub
		self.loader_sub
		self.camera_sub

		self.

    

class Motor:
	name = "none"
	velocity = 0.0
	p = 0.0
	i = 0.0
	d = 0.0

	def __init__(self, name, vel=0.0, p=0.0, i=0.0, d=0.0):
		self.name = name
		self.vel = vel
		self.p = p
		self.i = i
		self.d = d

def main():

	pong = Pong_System()

	while(True):

		# Check game status

	# create a central ros node that subscribes to 
	# overall pong state

	# when the pong game state changes		


if __name__ == '__main__':
	main()