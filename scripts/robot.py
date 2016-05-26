#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, Bool
from cse_190_assi_3.msg import AStarPath, PolicyList
import json
import image_util
from read_config import read_config
import astar as a

class Robot():
	def __init__(self):
		rospy.init_node('Robot')
		print ("inside Robot.py")
		ast = a.RAstar()
		self.simPub = rospy.Publisher("/sim_complete", Bool, queue_size=10)
		rospy.sleep(5)
		self.simPub.publish(True)               
 
if __name__ == '__main__':
	r = Robot()

