#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import sys
from scipy.misc import imread
from PIL import Image
from math import floor
import pickle
from tf.transformations import euler_from_quaternion
from math import sin, cos, sqrt

#def callback(msg):

	#global data_list1
	#global data_list2
	#global i

	#if (i < 50):
		#data_list1.append(msg.pose.pose.position.x)
		#data_list2.append(msg.pose.pose.position.y)
		#i += 1
	#else:
		#i = 0
		
		#fig = plt.figure(1)
		#ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])
		#ax2.plot(data_list1, data_list2, 'k', lw=1.0)
		#ax2.plot(data_list1[0], data_list2[0], marker='o', markerfacecolor='r', markersize=10.0)
		#ax2.plot(data_list1[-1], data_list2[-1], marker='^', markerfacecolor='b', markersize=10.0)
		#plt.ion()
		#plt.show()
		#plt.pause(0.1)
		#plt.clf()


def listener(topic_list, N):
	
	rospy.init_node('signal_visualizor', anonymous=True)
	# x, y data
	data_list1 = [[] for x in xrange(N)]
	data_list2 = [[] for x in xrange(N)]
	# vx, vy data
	data_list3 = [[] for x in xrange(N)]
	data_list4 = [[] for x in xrange(N)]
	# yaw, yawrate data
	data_list5 = [[] for x in xrange(N)]
	data_list6 = [[] for x in xrange(N)]
	
	count = 0
	
	print(" ###############################################")
	print(" -- red circle for starting point")
	print(" -- Blue Circle, Black Line for 1st target ")
	print(" -- Blue up-Triangle, Red Line for 2nd target ")
	print(" -- Blue square, Blue Line for 3rd target ")
	print(" -- Blue pentagon, Green Line for 4th target ")
	print(" -- for more lines please check the code for info")
	print(" -- x, y position data and vx vy yawrate data are saved in record.pkl " )
	print(" ############################################### ")
	
	marker_list = ['o', '^', 's', 'p', '8', 'h', 'x']
	color_list  = ['k', 'r', 'b', 'g', 'y', 'c', 'm']
	
	img = imread('/home/el2425/catkin_ws2/src/rcv_estimation/gps_show/scripts/3.png')
	
	im = Image.open('/home/el2425/catkin_ws2/src/rcv_estimation/gps_show/scripts/3.png')
	width, height = im.size
	print width, height
	width = float(width)
	height = float(height)
	
	ending = False
	
	dire_path = '/home/el2425/catkin_ws2/src/rcv_estimation/gps_show/scripts/'
	
	while(True):
		
		try:
			for k in range(N):
				# print "waiting for " + topic_list[k]
				try:
					data = rospy.wait_for_message(topic_list[k], Odometry, timeout=10)
				except rospy.ROSException:
					print "no more data in"
					ending = True
					break
				
				data_list1[k].append(data.pose.pose.position.x)
				data_list2[k].append(data.pose.pose.position.y)
				
				data_list3[k].append(data.twist.twist.linear.x)
				data_list4[k].append(data.twist.twist.linear.y)
				
				data_list5[k].append(data.twist.twist.angular.z)
				
				quaternion = data.pose.pose.orientation
				euler_angle = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
				data_list6[k].append(euler_angle[2])
			
			count = count + 1
			if count < 20:
				continue
			else:
				count = 0
				
				fig = plt.figure(1)
				ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])
				# ax2.set_xlim(-200, 50)
				# ax2.set_ylim(-50, 200)
				# print img.shape()
				
				# scale = 250.0
				# left = -199.0
				# right = left + scale
				# bottom = -45
				# top =  bottom + floor((height/width) * scale)
				
				# ax2.imshow(img, zorder=0, extent=[left, right, bottom, top])
				
				for k in range(N):
					ax2.plot(data_list1[k], data_list2[k], color=color_list[k], lw=1.0)
					ax2.plot(data_list1[k][0], data_list2[k][0], marker='o', markerfacecolor='r', markersize=10.0)
					ax2.plot(data_list1[k][-1], data_list2[k][-1], marker=marker_list[k], markerfacecolor='b', markersize=10.0)
					arrow_l = sqrt(abs(data_list1[k][0] - data_list1[k][-1])**2 +  abs(data_list2[k][0] - data_list2[k][-1])**2)/10.0
					if k == 0:   # estimated
						arrow_x = arrow_l * cos(data_list6[k][-1] + 3.14159265359/2.0)
						arrow_y = arrow_l * sin(data_list6[k][-1] + 3.14159265359/2.0)
						ax2.arrow(data_list1[0][-1], data_list2[0][-1], arrow_x, arrow_y, head_width=arrow_l/3.0, head_length=arrow_l/10.0, fc='k', ec='k')
						
						speed = sqrt(data_list3[k][-1]**2 + data_list4[k][-1]**2)
						arrow_x = arrow_l * data_list3[k][-1] / speed
						arrow_y = arrow_l * data_list4[k][-1] / speed
						ax2.arrow(data_list1[0][-1], data_list2[0][-1], arrow_x, arrow_y, head_width=arrow_l/3.0, head_length=arrow_l/10.0, fc='y', ec='y')
						
						
					elif k == 2:   # regression yaw
						arrow_x = arrow_l * cos(data_list6[k][-1])
						arrow_y = arrow_l * sin(data_list6[k][-1])
						ax2.arrow(data_list1[0][-1], data_list2[0][-1], arrow_x, arrow_y, head_width=arrow_l/3.0, head_length=arrow_l/10.0, fc='b', ec='b')
					elif k == 3:   # imu yaw
						arrow_x = arrow_l * cos(data_list6[k][-1])
						arrow_y = arrow_l * sin(data_list6[k][-1])
						ax2.arrow(data_list1[0][-1], data_list2[0][-1], arrow_x, arrow_y, head_width=arrow_l/3.0, head_length=arrow_l/10.0, fc='r', ec='r')
					
					#print "angle" + str(data_list6[k][-1])
					#print "cos,   sin"
					#print cos(data_list6[k][-1]), sin(data_list6[k][-1])
					#print "start, end"
					#print data_list1[k][-1], data_list2[k][-1], arrow_x, arrow_y
					
					

				plt.ion()
				plt.show()
				plt.pause(0.1)
				if ending:
					break
				else:
					plt.clf()
					
		except KeyboardInterrupt:
			print "end"
			sys.exit()
	
	f = open(dire_path + 'record_with_yaw.pkl', 'wb')
	print "writing pickle file"
	pickle.dump([data_list1, data_list2, data_list3, data_list4, data_list5, data_list6], f)
	f.close()
	print "done"
	plt.pause(100)
	exit()
	

if __name__ == '__main__':
	
	topic_list = rospy.get_param("/odom_topic_list")
	
	"""
	if len(sys.argv) < 2:
		print("usage: my_node.py odom_topic1 odom_topic2 ...")
	else:
		i = 0
		N = len(sys.argv) - 1
		topic_list = []
		for k in range(N):
			topic_list.append(sys.argv[k+1])
		
		print topic_list
		print "-------------"
		
		listener(topic_list, N)
	"""
	i = 0
	N = len(topic_list)
	print topic_list
	print "-------------"
	
	listener(topic_list, N)
