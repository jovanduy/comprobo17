#!/usr/bin/env python

""" ROS node to stop Neato when bump sensor is triggered """

from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
import rospy

class EmergencyStopNode(object):
	def __init__(self):
		rospy.init_node('emergencyStop')
		self.r = rospy.Rate(5)
		rospy.Subscriber('/bump', Bump, self.process_bump)
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.has_bumped = False
		
		
	
	def process_bump(self, m):
		if m.leftFront or m.leftSide or m.rightFront or m.rightSide:
			self.has_bumped = True
	def run(self):
		while not rospy.is_shutdown():
			if self.has_bumped:
				# stop robot
				twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)) 
			else:
				# go forward
				twist = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)) 
				
			
			self.publisher.publish(twist)
			self.r.sleep()

emergency_stop_node = EmergencyStopNode()
emergency_stop_node.run()
