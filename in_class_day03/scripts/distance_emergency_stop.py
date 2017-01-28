#!/usr/bin/env python

""" ROS node to stop Neato when bump sensor is triggered """

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import rospy

class DistanceStopNode(object):
	def __init__(self):
		rospy.init_node('distanceStop')
		self.r = rospy.Rate(5)
		rospy.Subscriber('/scan', LaserScan, self.process_scan)
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.max_distance = 1.0 # m away
		self.too_close = False
		
		
	
	def process_scan(self, m):
		if m.ranges[0] < self.max_distance:
			self.too_close = True
	def run(self):
		while not rospy.is_shutdown():
			if self.too_close:
				# stop robot
				twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)) 
			else:
				# go forward
				twist = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)) 
				
			
			self.publisher.publish(twist)
			self.r.sleep()

distance_stop_node = DistanceStopNode()
distance_stop_node.run()
