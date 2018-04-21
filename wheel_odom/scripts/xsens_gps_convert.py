#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from scipy.optimize import curve_fit
from numpy import linalg as LA
from nav_msgs.msg import Odometry

def reg_func(x, a, b, c, d):
	return a * np.power(x, 3) + b * np.power(x, 2) + c * x + d

class gps_converter:
	def __init__(self):
		self.speed = 0
		self.pub_gps = rospy.Publisher('/xsens_gps_fix', NavSatFix, queue_size=1)
		self.sub_gps = rospy.Subscriber("/communication/dspace/xsens_navsat", NavSatFix, self.callback, queue_size=1)
		self.sub_gps_local = rospy.Subscriber("/odometry/gps", Odometry, self.gps_local_cb, queue_size=1)
		self.sub_vel = rospy.Subscriber("/communication/dspace/avg_speed", Float32, self.readvel, queue_size=1)
		self.gps_cov = rospy.get_param("/xsens_gps_cov")
		self.variant_gps_cov = self.gps_cov
		self.past_gps_x = []
		self.past_gps_y = []
		self.countdown = 5
		
	def run(self):
		rospy.spin()
		

	def readvel(self, msg):
		self.speed = msg.data


	def gps_local_cb(self, msg):
		if self.speed > 1.0:
			# error = 3e-6
			error = 2
			if len(self.past_gps_x) < 10:
				self.past_gps_x.append(msg.pose.pose.position.x)
				self.past_gps_y.append(msg.pose.pose.position.y)
			else:
				self.past_gps_x.pop(0)
				self.past_gps_y.pop(0)
				self.past_gps_x.append(msg.pose.pose.position.x)
				self.past_gps_y.append(msg.pose.pose.position.y)
				self.countdown = self.countdown - 1
				if self.countdown == 0:
					r_x = np.array(self.past_gps_x)
					r_y = np.array(self.past_gps_y)
					# print r_x
					# print r_y
					try:
						dr_x = np.diff(r_x)
						dr_y = np.diff(r_y)
						# print dr_x
						dr = np.divide(dr_y, dr_x)
						error = np.std(dr)/abs(np.mean(dr))
						print("smoothness error "+str(error))
					except:
						print("regression failed")
					self.countdown = 5
		else:
			error = 2
			self.past_gps_x = []
			self.past_gps_y = []
		
		if self.speed < 0.1 :
			# self.variant_gps_cov = [1000000, 0, 0, 0, 1000000, 0, 0, 0, 1000000]
			self.variant_gps_cov = self.gps_cov
		else:
			# if error < 1e-5:
			if error < 2.5:
				self.variant_gps_cov = self.gps_cov
				if error < 1.0:
					# self.variant_gps_cov = [0.8, 0, 0, 0, 0.8, 0, 0, 0, 1.0]
					self.variant_gps_cov = self.gps_cov
			else:
				self.variant_gps_cov = [5.0, 0, 0, 0, 5.0, 0, 0, 0, 4.0]
				# self.variant_gps_cov = self.gps_cov


	def callback(self, msg):
		temp = msg
		temp.header.frame_id = 'trimble'
		temp.position_covariance = self.variant_gps_cov 
		
		# UNKNOWN = 0 APPROXIMATED = 1 DIAGONAL_KNOWN = 2 KNOWN = 3
		temp.position_covariance_type = 1
		self.pub_gps.publish(temp)


if __name__ == '__main__':
	
	rospy.init_node('xsens_gps_convert', anonymous=True)
	
	converter = gps_converter()
	converter.run()

