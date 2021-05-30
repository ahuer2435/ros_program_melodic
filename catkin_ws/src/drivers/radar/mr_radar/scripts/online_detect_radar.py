#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#---------------------------------------------------------------------------#
# The following code is used to parse the radar data to verify if an obstacle
# is present in front of the car. The data is read from the /radar_data topic
# and is then parsed through using the flag_counter function. In the event of 
# a set amount of detections (threshold variable found in bin_flag) a binary 
# flag is published to the /radar_obstacle topic.
#---------------------------------------------------------------------------#

# import ROS libraries
import rospy

# import message type
from mr_radar.msg import Radar_Data
from std_msgs.msg import Int8

# import module for arrays
import numpy as np 

# function for detection check
from detection import bin_flag, flag_counter 



class online_detect_radar(object):

	def __init__(self):
		#-----------------------Publisher/Subscriber-----------------------#
		rospy.Subscriber("/radar_data", Radar_Data, self.get_info) #subscriber to radar data
		self.brake = rospy.Publisher("/radar_obstacle", Int8, queue_size = 1000) #publisher for binary flag topic
		#-----------------------Constants-----------------------#
		self.flag_counter = 0 #no. of detections inside boundary zone
		self.ranges = np.zeros(10) #ranges [m]
		self.azimuths = np.zeros(10) #azimuth [rad]
		self.elevations = np.zeros(10) #elevation [rad]
		self.rel_vels = np.zeros(10) #relative velocity [m/s]
		self.rcss = np.zeros(10) #Radar Cross Section [dbm^2]
		self.range_vars = np.zeros(10) #range variance
		self.az_vars = np.zeros(10) #azimuth variance
		self.el_vars = np.zeros(10) #elevation variance
		self.vel_vars = np.zeros(10) #velocity variance
		self.snrs = np.zeros(10) #Signal-Noise Ratio [dbr]
		self.time = 0 #time of packet
		self.updated = False #update boolean

	# Callback to store published radar data
	def get_info(self,msg):
		if not(self.updated):
			self.time = msg.header.stamp #packet time
			self.ranges = np.array(msg.range) #ranges [m]
			self.azimuths = np.array(msg.azimuth) #azimuth [rad]
			self.elevations =np.array(msg.elevation) #elevation [rad]
			self.rel_vels = np.array(msg.rel_vel) #relative velocity [m/s]
			self.rcss = np.array(msg.rcs) #Radar Cross Section [dbm^2]
			self.range_vars = np.array(msg.range_var) #range variance
			self.az_vars = np.array(msg.az_var) #azimuth variance
			self.el_vars = np.array(msg.el_var) #elevation variance
			self.vel_vars = np.array(msg.vel_var) #velocity variance
			self.snrs = np.array(msg.snr) #Signal-Noise Ratio [dbr]
			self.updated = True #new information received
		return
	
	# Publisher for binary flag
	def pub(self,flag_raised):
		self.brake.publish(flag_raised)
		return

			
if __name__ == '__main__':
	rospy.init_node('Detection') #create node
	# 实例化online_detect_radar对象detector，调用__init__函数，创建发布者和订阅者
	detector = online_detect_radar() #create instance
	flag_raised = 0 #default value
	try:
		while not rospy.is_shutdown():
			# 当订阅话题有数据时，detector.updated为true，if成立。
			if detector.updated: #when a new packet is published on /radar_data
				#call detection function
				# 根据订阅到的消息，计算目标个数detector.flag_counter
				(detector.flag_counter) = flag_counter(detector.ranges, detector.azimuths, detector.elevations, detector.rcss, detector.rel_vels, detector.flag_counter,detector.snrs)    
				# 将detector.flag_counter转化为布尔型。
				flag_raised = bin_flag(detector.flag_counter)
				# detector.updated置为False，等待下次订阅话题的数据到来。 
				detector.updated = False
			#if verified obstacle
			# 当检测到目标时，flag_raised为True，将之发布出去。
			if (flag_raised):
				detector.pub(flag_raised) #publish to binary topic
				flag_raised = 0
				detector.flag_counter = 0
	except KeyboardInterrupt: #in the event of keyboard interrupt shutdown the socket connection
		rospy.loginfo('Radar shutting down.')
