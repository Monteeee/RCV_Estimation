#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from math import cos, sin, tan, sqrt, atan2, atan
import time
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from matplotlib import pyplot as plt



class wheel_odom_conv:
	
	def __init__(self):
		
		# ad-hoc solution for separate publishing of wheel data
		self.xyyaw = Point()
		self.odom_msg = Odometry()
		self.fl_str = 0
		self.fr_str = 0
		self.rl_str = 0
		self.rr_str = 0
		self.avg_vel = 0
		self.curv = 0
		self.msg_state = [0, 0, 0, 0, 0, 0, 0]
		
		# odometry info ---------------------
		self.last_t = time.time()
		self.x_global = 0
		self.y_global = 0
		self.last_omega = 0
		self.last_v = 0
		self.yaw_global = 0
		self.euler = []
		
		self.time_stamp = Time()
		
		self.init_flag = True
		
		self.sub_fl_str_ = rospy.Subscriber("/communication/dspace/flwheel_steering", Float32, self.cb1, queue_size=1)
		self.sub_fr_str_ = rospy.Subscriber("/communication/dspace/frwheel_steering", Float32, self.cb2, queue_size=1)
		self.sub_rl_str_ = rospy.Subscriber("/communication/dspace/rlwheel_steering", Float32, self.cb3, queue_size=1)
		self.sub_rr_str_ = rospy.Subscriber("/communication/dspace/rrwheel_steering", Float32, self.cb4, queue_size=1)
		self.sub_avg_vel_ = rospy.Subscriber("/communication/dspace/avg_speed", Float32, self.cb5, queue_size=1)
		self.sub_curv = rospy.Subscriber("/communication/dspace/curvature", Float32, self.cb6, queue_size=1)
		self.sub_time = rospy.Subscriber("/communication/dspace/imu", Imu, self.cb7, queue_size=1)
		
		self.pub_xyyaw_test = rospy.Publisher("/xyyaw_test", Point, queue_size=1)
		self.pub_odom_test = rospy.Publisher("/odom_test", Odometry, queue_size=1)
		# -----------------------------------------------------
		
		self.sub_wheel_data_all_ = rospy.Subscriber("/communication/dspace/wheel_data_all", Float32MultiArray, self.callback, queue_size=1)
		self.pub_wheel_odometry_ = rospy.Publisher('Wheel_Odometry', Odometry, queue_size=1)
		
		self.wheel_data = list()
		
		# for some visualization --------------------
		self.sample_count = 10
		self.point_x = list()
		self.point_y = list()
		
		# for parameter tuning
		self.pose_cov = rospy.get_param("/wheel_pose_cov")
		self.twist_cov = rospy.get_param("/wheel_twist_cov")
	
	def beta_curv_trans(self, theta_fl,theta_fr,theta_rl,theta_rr,v):
		
		a = 1.0
		b = 1.0
		
		
		theta_f=(theta_fl+theta_fr)/2.0;
		theta_r=(theta_rl+theta_rr)/2.0;
		
		x=(a * tan(theta_r) + b * tan(theta_f)) / (tan(theta_r)-tan(theta_f))
		
		y=(a-x)/tan(theta_f)

		# print "delta r and f " + str(theta_r) + "  " + str(theta_f)
		# print "x and y " + str(x) + "  " + str(y) 

		R=sqrt(x*x+y*y)
		
		if y >= 0:
			K=-1/R
		else:
			K=1/R
		
		# this remains an issue about whether atan or atan2 should be used
		if x/y>0:
			beta=-abs(atan(x/y))
		else:
			beta=abs(atan(x/y))
		
		# print "beta" + str(beta)
		
		return [beta,K]
	
	
	def wheel_odometry(self, theta_fl, theta_fr, theta_rl, theta_rr, v):
		
		if self.init_flag:
			self.init_flag = False
			self.last_t = time.time()
			
			return [0, 0, 0]
			
		else:
			# print "really computing"
			
			t_now = time.time()
			
			output = self.beta_curv_trans(theta_fl, theta_fr, theta_rl, theta_rr, v)
			current_beta = output[0]
			current_K = output[1]
			
			omega = current_K * v
			
			# print "curvature" + str(current_K)	
			
			yaw_old = self.yaw_global
					
			self.yaw_global = self.yaw_global + (omega + self.last_omega)/2.0 * abs(t_now - self.last_t) 
			
			# print "yaw_old " + str(yaw_old)
			# print "yaw_global " + str(self.yaw_global)

			
			self.yaw_global = self.yaw_global + (current_beta) # not sure
			
			# print "current_beta " + str(current_beta)
			self.yaw_global = self.yaw_global % 6.28318
			
			# print "yaw_global " + str(self.yaw_global)
			
			if abs(yaw_old - self.yaw_global) < 3.14159:
				yaw_avg = (yaw_old + self.yaw_global) / 2.0
			else:
				yaw_avg = self.yaw_global
			
			tvx = ( sin( (theta_fl + theta_fr)/2.0 ) + sin( (theta_rl + theta_rr)/2.0 ) )/2.0 * (v + self.last_v)/2.0
			tvy =  ( cos( (theta_fl + theta_fr)/2.0 ) + cos( (theta_rl + theta_rr)/2.0 ) )/2.0 * (v + self.last_v)/2.0
			
			vx = (tvx*cos(self.imu_yaw) + tvy*cos(self.imu_yaw+3.14159265359/2.0) )
			vy = (tvx*sin(self.imu_yaw) + tvy*sin(self.imu_yaw+3.14159265359/2.0) )
			
			self.x_global = self.x_global + vx*abs(t_now - self.last_t)
			self.y_global = self.y_global + vy*abs(t_now - self.last_t)
			
			self.last_t = time.time()
			self.last_omega = omega
			self.last_v = v
			
			self.xyyaw.x = self.x_global
			self.xyyaw.y = self.y_global
			self.xyyaw.z = self.yaw_global
			
			self.pub_xyyaw_test.publish(self.xyyaw)
			
			# publish odometry message 
			self.odom_msg.header.stamp = self.time_stamp
			self.odom_msg.header.frame_id = "odom"
			self.odom_msg.child_frame_id = "base_link"
			
			quad_temp = quaternion_from_euler(0.0, 0.0, self.yaw_global)
			self.odom_msg.pose.pose.orientation.x = quad_temp[0]
			self.odom_msg.pose.pose.orientation.y = quad_temp[1]
			self.odom_msg.pose.pose.orientation.z = quad_temp[2]
			self.odom_msg.pose.pose.orientation.w = quad_temp[3]
			
			self.odom_msg.pose.pose.position.x = self.x_global
			self.odom_msg.pose.pose.position.y = self.y_global
			self.odom_msg.pose.pose.position.z = 0.0
			
			self.odom_msg.twist.twist.linear.x = vx
			self.odom_msg.twist.twist.linear.y = vy
			self.odom_msg.twist.twist.linear.z = 0.0
			
			self.odom_msg.twist.twist.angular.x = 0.0
			self.odom_msg.twist.twist.angular.y = 0.0
			self.odom_msg.twist.twist.angular.z = omega
			
			# x y z theta_x, theta_y, theta_z
			#self.odom_msg.pose.covariance = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
											 #0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 
											 #0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 
											 #0.0, 0.0, 0.0, 0.9, 0.0, 0.0, 
											 #0.0, 0.0, 0.0, 0.0, 0.9, 0.0, 
											 #0.0, 0.0, 0.0, 0.0, 0.0, 0.9]
			self.odom_msg.pose.covariance = self.pose_cov
			
			# v of x y z, angular velocity of theta_x theta_y theta_z						 
			#self.odom_msg.twist.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 
			                                  #0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 
			                                  #0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 
			                                  #0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 
			                                  #0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 
			                                  #0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
			self.odom_msg.twist.covariance = self.twist_cov
			
			self.pub_odom_test.publish(self.odom_msg)
			
			# if self.sample_count == 0:
			# self.point_x.append(self.x_global)
			# self.point_y.append(self.y_global)
			# self.sample_count = 10
			# else:
			# self.sample_count = self.sample_count - 1
			
			return [self.x_global, self.y_global, self.yaw_global]
	
	
	def cb1(self, msg):
		
		# print "rec 1"
		
		self.fl_str = msg.data
		self.msg_state[0] = 1
		
		if sum(self.msg_state) < 7:
			return
		else:
			self.compute_odom()
	
	def cb2(self, msg):
		
		# print "rec 2"
		
		self.fr_str = msg.data
		self.msg_state[1] = 1
		
		if sum(self.msg_state) < 7:
			return
		else:
			self.compute_odom()
	
	def cb3(self, msg):
		
		# print "rec 3"
		
		self.rl_str = msg.data
		self.msg_state[2] = 1
		
		if sum(self.msg_state) < 7:
			return
		else:
			self.compute_odom()

	def cb4(self, msg):
		
		# print "rec 4"
		
		self.rr_str = msg.data
		self.msg_state[3] = 1
		
		if sum(self.msg_state) < 7:
			return
		else:
			self.compute_odom()
		
	def cb5(self, msg):
		
		# print "rec 5"
		
		self.avg_vel = msg.data
		self.msg_state[4] = 1
		
		if sum(self.msg_state) < 7:
			return
		else:
			self.compute_odom()
	
	def cb6(self, msg):
		
		# print "rec 6"
		
		self.curv = msg.data
		self.msg_state[5] = 1
	
		if sum(self.msg_state) < 7:
			return
		else:
			self.compute_odom()
	
	def cb7(self, msg):
		self.time_stamp = msg.header.stamp
		quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
		
		# print "quaternion " + str(quaternion)
		
		euler = euler_from_quaternion(quaternion)
		
		# print "euler " + str(euler)
		
		self.imu_yaw = euler[2]
		
		self.msg_state[6] = 1
		if sum(self.msg_state) < 7:
			return
		else:
			self.compute_odom()
	
	def callback(self, msg):
		wheel_data = msg.data
		
	def compute_odom(self):
		
		# print "-------------"
		
		self.wheel_odometry(self.fl_str, self.fr_str, self.rl_str, self.rr_str, self.avg_vel)
		self.msg_state = [0, 0, 0, 0, 0, 0, 0]

	def listener(self):
		rospy.spin()
	

if __name__ == '__main__':
	
	rospy.init_node('wheel_odom_conv', anonymous=True)

	converter = wheel_odom_conv()
	# data_list1 = []
	# data_list2 = []
	converter.listener()
