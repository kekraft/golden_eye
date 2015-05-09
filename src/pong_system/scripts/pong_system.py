#!/usr/bin/env python

import roslib; roslib.load_manifest('pong_system')
import rospy

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
from matplotlib import pyplot as plt
import copy

# IS THERE A WAY TO FORCE TARGETTING?
# HOW DO WE STEP IN MANUALLY TO DO TARGETTING?


from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class Pong_System:
	def __init__(self):
		self.state = Game_State.SETUP
		self.targeted = False
		self.manual = False

		######## Do calibration routine here ###########
		######## Do inital targetting ##################

		self.motor_a = Motor("motor_a")
		self.motor_b = Motor("motor_b")
		self.motor_c = Motor("motor_c")

		# Subscribers to each subsystem state
		self.game_state_sub = rospy.Subscriber('/game/state', String, self.state_cb)
		self.launcher_state_sub = rospy.Subscriber('/launcher/state', String, self.state_cb)
		self.loader_state_sub = rospy.Subscriber('/loader/state', String, self.state_cb)
		self.vision_state_sub = rospy.Subscriber('/vision/state', String, self.state_cb)

		# subscribe to visions output
		self.cup_loc_sub = rospy.Subscriber('/vision/cup_location', Vector3, self.cup_loc_cb)

		# Publishers for each topic that the subsystems listen to
		# Loader cmd (True = Load, False = Do nothing)
		self.loader_pub = rospy.Publisher('/loader/load_cmd', Bool, queue_size=10)

		# launcher velocity and pid commands
		# Vector3.x = motor a speed, Vector3.y = motor b speed, Vector3.z = motor c speed
		self.launcher_motor_vel_pub = rospy.Publisher('/launcher/motor_vel', Vector3, queue_size=10)
		# Vector3.x = kp, Vector3.y = ki, Vector3.z = kd
		self.launcher_pid_pub = rospy.Publisher('/launcher/pid_val', Vector3, queue_size=10)

		# subscribe to game state
		# a true message sent to this topic means we are on offense and should shoot
		# a false message sent to this topic means we are on defense and should do targeting
		self.game_offensive_sub = rospy.Subscriber('/game/offense', Bool, self.game_side_cb)


	def state_cb(self, msg):
		# print out the data
		print msg.data

		# log the data
		rospy.loginfo(msg)

	def cup_loc_cb(self, msg):
		''' This is called whenever a new cup position is given by the vision system.
			 What happens here is the pixel coords are translated to world coordinates
			 	and that is transformed to a motor velocity. Each motor velocity is set 
			 	appropriately
		'''
		self.cup_loc = (msg.x, msg.y, msg.x)
		X = msg.x
		y = msg.y
		z = msg.z

		# transform pixel cup location to world location


		# transform world location to motor velocity

		self.targeted = True

	def game_side_cb(self, msg):
		''' Reading True means we are on offense.
				If on offense, load a ball
			Reading False means we are on defense.
				If on defense, go ahead and set the motors to 
				 spin at the targeted cup
		'''
		if (msg.data is True) and (self.state is not Game_State.SETUP):
			rospy.loginfo("On offense.")

			if self.targeted is False:
				# force to target the cups
				pass

			# motors' velocities should already be set to hit targeted cup
			# load ball
			rospy.loginfo("Sending load cmd")
			cmd = Bool()
			cmd.data = True
			self.loader_pub.publish(cmd)

			self.targeted = False

		else:  

			# target and start motors
			if not self.targeted:
				# force targetting
				pass

			# command motors based on target
			cmd = Vector3()
			cmd.x = self.motor_a.vel
			cmd.y = self.motor_b.vel
			cmd.z = self.motor_c.vel
			self.launcher_motor_vel_pub.publish(cmd)



class Motor:
	name = "none"
	vel = 0.0
	p = 0.0
	i = 0.0
	d = 0.0

	def __init__(self, name, vel=0.0, p=0.0, i=0.0, d=0.0):
		self.name = name
		self.vel = vel
		self.p = p
		self.i = i
		self.d = d

class Game_State:
	SETUP = 0
	OFFENSE = 1
	DEFENSE = 2

def main():

	rospy.init_node('pong_system')

	pong = Pong_System()
	
	rospy.spin()

if __name__ == '__main__':
	main()