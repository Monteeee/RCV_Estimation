#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
from numpy import linalg as LA
import sys
from scipy.misc import imread
from PIL import Image
from math import floor
import pickle
from math import sin, cos
from scipy.optimize import curve_fit
from scipy.misc import derivative
import time
from math import atan, atan2, sqrt

def reg_func(x, a, b, c, d):
	return a * np.power(x, 3) + b * np.power(x, 2) + c * x + d


if __name__ == '__main__':
	dire_path = '/home/el2425/catkin_ws2/src/rcv_estimation/gps_show/scripts/'
	
	# ############# the record file you want to use ####################
	file_number = 29
	record_file_name = 'record_with_yaw_' + str(file_number) + '.pkl'
	
	f = open(dire_path + record_file_name, 'rb')
	x_list, y_list, vx_list, vy_list, yawrate_list, yaw_list = pickle.load(f)
	f.close()
	
	N = 4
	
	handle = range(N)
	labels = ['ros_ekf', 'gps_xsens', 'gps_trimble', 'dspace_ekf'] 
	msize = 5.0
	
	marker_list = ['o', '^', 's', 'p', '8', 'h', 'x']
	color_list  = ['k', 'r', 'b', 'g', 'y', 'c', 'm']
	
	# ##### control verbose ########
	verbose_draw_route = 1
	verbose_draw_velocity_record = 0
	verbose_draw_yaw = 0
	verbose_draw_acc = 0
	verbose_draw_vel = 0
	
	verbose_draw_fit = 0
	
	img = imread('/home/el2425/catkin_ws2/src/rcv_estimation/gps_show/scripts/3.png')
	
	im = Image.open('/home/el2425/catkin_ws2/src/rcv_estimation/gps_show/scripts/3.png')
	width, height = im.size
	width = float(width)
	height = float(height)
	
	if verbose_draw_fit == 1:
		print len(x_list[0])
		print len(y_list[0])
		
		start = 500
		step = 5
		end = 550
		for increment in xrange(0, end-start, step):
			upper = start + increment + step
			lower = upper - 15
			
			reg_test_x = np.array(x_list[0][lower:upper])
			reg_test_y = np.array(y_list[0][lower:upper])
			
			t_series = np.linspace(0.0, 10.0, len(reg_test_x))
			
			po1, pc1 = curve_fit(reg_func, t_series, reg_test_x)
			po2, pc2 = curve_fit(reg_func, t_series, reg_test_y)
			
			error = sqrt(LA.norm(reg_func(t_series, *po2) - reg_test_y)**2 + 
							LA.norm(reg_func(t_series, *po1) - reg_test_x)**2)
			print str(error)
			
			g1 = lambda x: po1[0]*x**3 + po1[1]*x**2 + po1[2]*x + po1[3]
			g2 = lambda x: po2[0]*x**3 + po2[1]*x**2 + po2[2]*x + po2[3]
			
			dx = derivative(g1, t_series[-2])
			dy = derivative(g2, t_series[-2])
			
			angle_temp = atan2(dy, dx)
			yaw_reg = angle_temp
			
			#popt, pcov = curve_fit(reg_func, reg_test_x, reg_test_y)
			#g = lambda x: popt[0]*x**3 + popt[1]*x**2 + popt[2]*x + popt[3]
			#dx = derivative(g, reg_test_x[-1])
			
			#angle_temp = atan(dx)
					
			 ######## replace this with ending point vector
			#vec_1 = np.array([cos(angle_temp), sin(angle_temp)])
			#vec_3 = np.array([reg_test_x[-1]-reg_test_x[-2], reg_test_y[-1]-reg_test_y[-2]])
			
			#yaw_reg = angle_temp
			
			#inn1 = np.inner(vec_1, vec_3)

			#if inn1 > 0:
				#yaw_reg = angle_temp
			#else:
				#yaw_reg = angle_temp + 3.14159265359
			
			## print popt
			#error = LA.norm(reg_func(reg_test_x, *popt) - reg_test_y)
			#print "angle " + str(yaw_reg) + " error " + str(error)
			
			plt.plot(reg_test_x, reg_test_y, 'b-', label='data')
			plt.plot(reg_func(t_series, *po1), reg_func(t_series, *po2), 'r-', label='fit')
			# plt.plot(reg_test_x, reg_func(reg_test_x, *popt), 'r-', label='fit')
			arrow_l = 1.0
			arrow_x = arrow_l * cos(yaw_reg)
			arrow_y = arrow_l * sin(yaw_reg)
			if error < 1.0:
				plt.arrow(reg_test_x[-1], reg_test_y[-1], arrow_x, arrow_y, head_width=arrow_l/2.0, head_length=arrow_l/5.0, fc='b', ec='b')
						
			plt.legend()
		
		#if popt[0] > 0.01:
			#print np.divide(np.sqrt(np.diag(np.fabs(pcov))), np.power(popt, 2))
		#else:
			#print np.divide(np.sqrt(np.diag(np.fabs(pcov))[1:4]), np.power(popt, 2)[1:4])
		
		# print reg_test_x[-1]
		# print dx
		

	
	# ==================================================================
	# draw the result on the map image (only for short_with_circles.bag)
	
	if verbose_draw_route == 1:
		fig = plt.figure(1)
		ax2 = fig.add_axes([0.1, 0.1, 0.8, 0.8])
		# ax2.set_xlim(-200, 50)
		# ax2.set_ylim(-50, 200)

		scale = 250.0
		left = -199.0
		right = left + scale
		bottom = -45
		top =  bottom + floor((height/width) * scale)

		# ax2.imshow(img, zorder=0, extent=[left, right, bottom, top])

		for k in [0, 2]:
			print(len(x_list[k]))
			
			ax2.plot(x_list[k], y_list[k], color=color_list[k], lw=1.0)
			ax2.plot(x_list[k][0], y_list[k][0], marker='o', markerfacecolor='r', markersize=10.0)
			ax2.plot(x_list[k][-1], y_list[k][-1], marker=marker_list[k], markerfacecolor='b', markersize=10.0, label='route ' + labels[k])
		
		ax2.legend(bbox_to_anchor=(0.1, 1), loc=2, borderaxespad=0.)
	
	# ==================================================================
	# compute vx, vy for rcv_odom in its global frame
	
	if verbose_draw_velocity_record == 1:
		vx_rcv = np.zeros(len(yaw_list[3]))
		vy_rcv = np.zeros(len(yaw_list[3]))
		for i in range(len(yaw_list[3])):
			vx_rcv[i] = vx_list[3][i] * cos(yaw_list[3][i])
			vy_rcv[i] = vx_list[3][i] * cos(yaw_list[3][i])

		vx_list[3] = vx_rcv
		vy_list[3] = vy_rcv

		# draw the velocity record
		
		fig2 = plt.figure(2)
		ax3 = fig2.add_axes([0.1, 0.1, 0.8, 0.8])
		
		for k in range(N):
			t_list = np.linspace(0.0, 100.0, len(vx_list[k]))
			handle[k] = ax3.plot(t_list, vx_list[k], color=color_list[k], marker=marker_list[k], label='vx '+labels[k], lw=1.0, markersize=msize)
		
		ax3.legend()
		
		fig3 = plt.figure(3)
		ax4 = fig3.add_axes([0.1, 0.1, 0.8, 0.8])
		
		for k in range(N):
			t_list = np.linspace(0.0, 100.0, len(vy_list[k]))
			handle[k] = ax4.plot(t_list, vy_list[k], color=color_list[k], marker=marker_list[k], label='vy '+labels[k], lw=1.0, markersize=msize)
		
		ax4.legend()
	
	
	# ==================================================================
	### draw yaw angle
	
	if verbose_draw_yaw == 1:	
		fig6 = plt.figure(6)
		ax7 = fig6.add_axes([0.1, 0.1, 0.8, 0.8])
		
		for k in [0, 2, 3]:
			# t_list = np.linspace(0.0, 100.0, len(yaw_list[k]))
			t_list = range(len(yaw_list[k]))
			if k == 0 or k == 2:
				handle[k] = ax7.plot(t_list, np.array(yaw_list[k]), color=color_list[k], marker=marker_list[k], label='yaw '+labels[k], lw=1.0, markersize=msize)
			else:
				handle[k] = ax7.plot(t_list, np.array(yaw_list[k]), color=color_list[k], marker=marker_list[k], label='yaw '+labels[k], lw=1.0, markersize=msize)
		ax7.legend()
	
	
	# ==================================================================
	### draw estimated vx, vy
	
	if verbose_draw_acc or verbose_draw_vel:
		vx_est = []
		vy_est = []
		ax_est = []
		ay_est = []
		
		for x_i in x_list:
			f = np.array(x_i, dtype=np.float)
			df = np.gradient(f)
			vx_est.append(df)
			ax_est.append(np.gradient(df))
			
		for y_i in y_list:
			f = np.array(y_i, dtype=np.float)
			df = np.gradient(f)
			vy_est.append(df)
			ay_est.append(np.gradient(df))
	
	# ==================================================================
	# draw estimated ACC
	if verbose_draw_acc == 1:
		
		fig4 = plt.figure(4)
		ax5 = fig4.add_axes([0.1, 0.1, 0.8, 0.8])
		
		# for k in range(N):
		for k in [0, 1, 3]:
			t_list = np.linspace(0.0, 100.0, len(ax_est[k]))
			handle[k] = ax5.plot(t_list, ax_est[k], color=color_list[k], marker=marker_list[k], label='vx_est '+labels[k], lw=1.0, markersize=msize)
		
		ax5.legend()
		
		fig5 = plt.figure(5)
		ax6 = fig5.add_axes([0.1, 0.1, 0.8, 0.8])
		
		# for k in range(N):
		for k in [0, 1, 3]:
			t_list = np.linspace(0.0, 100.0, len(ay_est[k]))
			handle[k] = ax6.plot(t_list, ay_est[k], color=color_list[k], marker=marker_list[k], label='vy_est '+labels[k], lw=1.0, markersize=msize)
		
		ax6.legend()
		
	# ==================================================================
	# draw estimated VEL	
	elif verbose_draw_vel == 1:
		
		fig4 = plt.figure(4)
		ax5 = fig4.add_axes([0.1, 0.1, 0.8, 0.8])
		
		# for k in range(N):
		for k in [0]:
			t_list = np.linspace(0.0, 100.0, len(vx_est[k]))
			handle[k] = ax5.plot(t_list, vx_est[k], color=color_list[k], marker=marker_list[k], label='vx_est '+labels[k], lw=1.0, markersize=msize)
		
		
		#for k in [0]:
			#t_list = np.linspace(0.0, 100.0, len(vx_list[k]))
			#handle[k] = ax5.plot(t_list, vx_list[k], color='b', marker=marker_list[k], label='vx '+labels[k], lw=1.0, markersize=msize)
		
		ax5.legend()
		
		fig5 = plt.figure(5)
		ax6 = fig5.add_axes([0.1, 0.1, 0.8, 0.8])
		
		# for k in range(N):
		for k in [0]:
			t_list = np.linspace(0.0, 100.0, len(vy_est[k]))
			handle[k] = ax6.plot(t_list, vy_est[k], color=color_list[k], marker=marker_list[k], label='vy_est '+labels[k], lw=1.0, markersize=msize)
		
		#for k in [0]:
			#t_list = np.linspace(0.0, 100.0, len(vy_list[k]))
			#handle[k] = ax6.plot(t_list, vy_list[k], color='b', marker=marker_list[k], label='vy '+labels[k], lw=1.0, markersize=msize)
		
		ax6.legend()
		
		
	
	plt.ion()
	plt.show()
	plt.pause(10000)
	
	
