#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix
import sys

def callback(msg):
	global pub_gps
	global gps_cov
	
	temp = msg
	temp.header.frame_id = 'trimble'
	# UNKNOWN = 0 APPROXIMATED = 1 DIAGONAL_KNOWN = 2 KNOWN = 3
	
	### something could be done here, based on kinematically impossible 
	### behavior of GPS position signal, increase the covariance size 
	### of the GPS
	
	temp.position_covariance = gps_cov
	temp.position_covariance_type = 1
	pub_gps.publish(temp)


if __name__ == '__main__':
	
	rospy.init_node("trimble_gps_convert", anonymous=True)
	
	gps_cov = rospy.get_param("/trimble_gps_cov")
	# new_topic_name = rospy.get_param("/new_gps_topic_name")
	# print "--- new name"
	# print new_topic_name 
	
	sub_gps = rospy.Subscriber("/fix", NavSatFix, callback, queue_size=1)
	pub_gps = rospy.Publisher("/trimble_gps_fix", NavSatFix, queue_size=1)
	
	rospy.spin()
