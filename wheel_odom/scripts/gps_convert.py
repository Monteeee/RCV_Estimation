#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix

def callback(msg):
	global pub_gps
	global gps_cov
	
	temp = msg
	temp.header.frame_id = 'trimble'
	temp.position_covariance = gps_cov
	temp.position_covariance_type = 3
	pub_gps.publish(temp)


if __name__ == '__main__':
	rospy.init_node('gps_convert', anonymous=True)
	
	gps_cov = rospy.get_param("/gps_cov")
	
	sub_gps = rospy.Subscriber("/fix", NavSatFix, callback, queue_size=1)
	pub_gps = rospy.Publisher('/gps_fix', NavSatFix, queue_size=1)
	
	rospy.spin()
