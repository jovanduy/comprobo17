#!/usr/bin/env python

""" ROS node to go forward until obstacle, then back, then turn left then go forward """

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan, Bump
from std_msgs.msg import Header
import rospy

class AvoidObstacleNode(object):
	def __init__(self):
		rospy.init_node('distanceStop')
		self.r = rospy.Rate(5)
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		rospy.Subscriber('/bump', Bump, self.process_bump)
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.distance_threshold = 1.0 # m away
		self.too_close = False
		self.has_bumped = False
		self.states = ['forward', 'backward', 'left']
		self.curr_state = self.states[0]
		
		
	def process_bump(self, m):
		if m.leftFront or m.leftSide or m.rightFront or m.rightSide:	
			self.has_bumped = True
	def process_scan(self, m):
		if m.ranges[0] < self.max_distance:
			self.too_close = True
	def run(self):
		while not rospy.is_shutdown():
			if self.curr_state == self.states[0]:
				# moving forward state
				if self.has_bumped:
					self.curr_state = self.states[1] 
				else:
					twist = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)) 
			elif self.curr_state == self.states[1]:
				# moving backward state
				if not self.too_close:
					self.curr_state = self.states[2]
					# record time
				else:
					twist = Twist(linear=Vector3(-1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)) 

			elif self.curr_state == self.states[2]:
				# turning left
				
			if self.too_close:
				# stop robot
							else:
				# go forward
				twist = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)) 
				
			
			self.publisher.publish(twist)
			self.r.sleep()

distance_stop_node = DistanceStopNode()
distance_stop_node.run()
