#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import sys

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
	data_list1 = [[] for x in xrange(N)]
	data_list2 = [[] for x in xrange(N)]
	count = 0
	
	print(" ###############################################")
	print(" -- red circle for starting point")
	print(" -- blue circle for first target ")
	print(" -- blue up triangle for 2nd target ")
	print(" -- blue square for 3rd target ")
	print(" -- blue pentagon for 4th target ")
	print(" -- for more lines please check the code for info")
	print(" ############################################### ")
	
	
	marker_list = ['o', '^', 's', 'p', '8', 'h', 'x']
	color_list  = ['k', 'r', 'b', 'g', 'y', 'c', 'm']
	
	while(True):
		for k in range(N):
			# print "waiting for " + topic_list[k]
			data = rospy.wait_for_message(topic_list[k], Odometry, timeout=10)
			data_list1[k].append(data.pose.pose.position.x)
			data_list2[k].append(data.pose.pose.position.y)
			
		count = count + 1
		if count < 20:
			continue
		else:
			count = 0
			
			fig = plt.figure(1)
			ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])
			# ax2.set_xlim(-150, 100)
			# ax2.set_ylim(-50, 300)
			
			for k in range(N):
				ax2.plot(data_list1[k], data_list2[k], color=color_list[k], lw=1.0)
				ax2.plot(data_list1[k][0], data_list2[k][0], marker='o', markerfacecolor='r', markersize=10.0)
				ax2.plot(data_list1[k][-1], data_list2[k][-1], marker=marker_list[k], markerfacecolor='b', markersize=10.0)
			
			plt.ion()
			plt.show()
			plt.pause(0.1)
			plt.clf()
			

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
