#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.pylab as plb
import time
import numpy as np
import math
from stargazer import StarGazer
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from stargazer.msg import (MarkerPose, MarkerPoses,
						   MarkerRawPose, MarkerRawPoses)
import copy

class PoseRepresent:

	def __init__(self):
		
		# ROS parameters
		# rospy.set_param('step', [])
		# rospy.set_param('count', 0)
		# self.timer = 0

		# Initialize ros node
		rospy.init_node('pose_represent', anonymous = True)

		# marker_id bufer
		self.prev_marker_id = None
		self.count = 0
		# self.ekf_x	= 0
		# self.ekf_y	= - 1
		# self.opti_x	= - 0.46
		# self.opti_y	= - 0.06

		# Subscriber
		rospy.Subscriber('marker_poses', PoseWithCovarianceStamped, self.markerCallback, queue_size=1)    # local pose
		rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, self.robotCallback, queue_size=1)    # global pose
		rospy.Subscriber('vrpn_client_node/RigidBody/pose_aligned', PoseStamped, self.optitrackCallback, queue_size=1)    # global pose
		rospy.Subscriber('odom', Odometry, self.odomCallback, queue_size=1)    # global pose
		rospy.Subscriber('imu', Imu, self.imuCallback, queue_size=1)    # global pose
		rospy.Subscriber('odometry/filtered', Odometry, self.ekfCallback, queue_size=1) # filtered pose
		

	def spin(self):
		rospy.spin()


	def markerCallback(self, msg):
		
		self.markerpose = msg
		self.marker_id_buffer = self.markerpose.header.frame_id
		# if self.marker_id_buffer == 


	def robotCallback(self, msg):
		
		self.robotpose = copy.deepcopy(msg)
		self.x_robot = self.robotpose.pose.pose.position.x
		self.y_robot = self.robotpose.pose.pose.position.y

		rotation = (self.robotpose.pose.pose.orientation.x,
					self.robotpose.pose.pose.orientation.y,
					self.robotpose.pose.pose.orientation.z,
					self.robotpose.pose.pose.orientation.w)
		self.robot_yaw, self.robot_degree = self.getAnglefromquaternion(rotation)
		self.poseGraph()


	def ekfCallback(self, msg):
		
		self.ekfpose = copy.deepcopy(msg)
		self.x_filtered = - self.ekfpose.pose.pose.position.x
		self.y_filtered = self.ekfpose.pose.pose.position.y


	def optitrackCallback(self, msg):
    	
		self.groundTruth = copy.deepcopy(msg)
		self.x_truth = self.groundTruth.pose.position.x
		self.y_truth = self.groundTruth.pose.position.y

		rotation = (self.groundTruth.pose.orientation.x,
					self.groundTruth.pose.orientation.y,
					self.groundTruth.pose.orientation.z,
					self.groundTruth.pose.orientation.w)
		self.ground_yaw, self.ground_degree = self.getAnglefromquaternion(rotation)
		
		# self.poseGraph()


	def odomCallback(self, msg):
    	
		self.odom = msg
		rotation = (self.odom.pose.pose.orientation.x,
					self.odom.pose.pose.orientation.y,
					self.odom.pose.pose.orientation.z,
					self.odom.pose.pose.orientation.w)
		self.odom_angle = self.getAnglefromquaternion(rotation)
		self.odom_yaw, self.odom_degree = self.getAnglefromquaternion(rotation)


	def imuCallback(self,msg):
    		
		self.imu = msg
		rotation = (self.imu.orientation.x,
					self.imu.orientation.y,
					self.imu.orientation.z,
					self.imu.orientation.w)
		self.imu_angle = self.getAnglefromquaternion(rotation)
		self.imu_yaw, self.imu_degree = self.getAnglefromquaternion(rotation)


	def poseGraph(self):
		'''
		draw the mesearued pose
		'''
		
		plt.ion()

		# print('Robot pose       = ',self.x_robot, self.y_robot)
		# print('Ground truth     = ',self.x_truth, self.y_truth)
		# print('Robot angle = ',self.robot_degree)
		# print('GroundTruth angle = ',self.ground_degree)
		# print('odom angle = ',self.odom_degree)
		# print('imu angular velocity = ', self.imu.angular_velocity.z)
		# print('imu angle = ',self.imu_degree)
		# print("filtered pose    = ",self.x_filtered, self.y_filtered)
		'''
		(x,y) coordinate
		'''
		# plt.plot(self.x_robot, self.y_robot, 'b.',
		# 		self.x_filtered, self.y_filtered, 'r.--',
		# 		self.x_truth, self.y_truth, 'k.:')
		# # blue for robot / red for filtered / black for truth 
		# plt.axis([-0.05, 0.16, -1.15, 2.0]) # x axis from -0.5 to 0.1, y axis from -2.0 to 2.0
		# plt.xlabel('x axis(m)')
		# plt.ylabel('y axis(m)')
		# plt.title('Straight line w constant velocity')
		'''
		x error
		'''
		self.count += 1
		print(self.count)
		diff_x = self.x_filtered - self.x_truth
		plt.plot(self.count, self.x_filtered, 'r.',
					self.count, self.x_truth, 'k.',
					self.count, self.x_robot, 'g.',
					self.count, diff_x, 'b.')
		plt.axis([0, 150, -0.05, 0.16])
		'''
		y error
		'''
		# diff_y = self.y_filtered - self.y_truth
		# plt.plot(self.count, self.y_filtered, 'r.',
		# 			self.count, self.y_truth, 'k.',
		# 			self.count, self.y_robot, 'g.',
		# 			self.count, diff_y, 'b.')
		# plt.plot(self.count, diff_y, 'b.')
		# plt.axis([0, 150, -1.15, 0.1]) # 150 count for y axis from -1.2 to 2
		plt.xlabel('count')
		plt.ylabel('x axis(m)')
		plt.title('Straight line w constant velocity')
		
		
		plt.grid(True)
		# plt.legend(loc = 'upper right')
		plt.draw()
		plt.pause(0.01)


	def getAnglefromquaternion(self, rotation):
    	
		euler_angle = euler_from_quaternion(rotation)
		yaw = euler_angle[2]
		if yaw < -math.pi:
    			yaw += 2 * math.pi
		if yaw > math.pi:
    			yaw -= 2 * math.pi
		degree = yaw * 180 / math.pi

		return yaw, degree


##  End of class ObstacleRepresent.


if __name__ == '__main__':

	try:
		node = PoseRepresent()
		node.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")
