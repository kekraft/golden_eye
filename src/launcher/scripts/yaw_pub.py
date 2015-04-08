#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
import random

def publisher():
	pub = rospy.Publisher('launcher/yaw', UInt16, queue_size=10)
	rospy.init_node('yaw_control', anonymous=False)
	rate = rospy.Rate(1) # 1 hz
	angle = 90
	while not rospy.is_shutdown():
		angle = random.randint(75,120)
		print "Angle: ", angle
		pub.publish(angle)


		rate.sleep()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
