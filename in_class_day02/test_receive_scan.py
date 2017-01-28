#!/user/bin/env python

""" This script is our first ROS node. Wel'll publish some messages """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy

def process_scan(m):
	print m.ranges[0]

rospy.init_node('test_receive_message')
rospy.Subscriber('/scan', LaserScan, process_scan)

r = rospy.Rate(2)
while not rospy.is_shutdown():
	r.sleep()

print "Node is finished"
