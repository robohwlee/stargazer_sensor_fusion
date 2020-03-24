#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.pylab as plb
import time
import numpy as np
from stargazer import StarGazer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from stargazer.msg import (MarkerPose, MarkerPoses,
						   MarkerRawPose, MarkerRawPoses)

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
		self.ekf_x	= - 0.30
		self.ekf_y	= - 1.40
		self.opti_x	= - 0.46
		self.opti_y	= - 0.06

		# Subscriber
		rospy.Subscriber('marker_poses', PoseWithCovarianceStamped, self.markerCallback, queue_size=1)    # local pose
		rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, self.robotCallback, queue_size=1)    # global pose
		rospy.Subscriber('vrpn_client_node/RigidBody/pose', PoseStamped, self.optitrackCallback, queue_size=1)    # global pose
		rospy.Subscriber('odometry/filtered', Odometry, self.ekfCallback, queue_size=1) # filtered pose
		

	def spin(self):
		rospy.spin()


	def markerCallback(self, msg):
		
		self.markerpose = msg
		self.marker_id_buffer = self.markerpose.header.frame_id
		# if self.marker_id_buffer == 


	def robotCallback(self, msg):
		
		self.x_robot = msg.pose.pose.position.x
		self.y_robot = msg.pose.pose.position.y
		self.robotpose = msg
		self.poseGraph()


	def ekfCallback(self, msg):
		
		self.x_filtered = msg.pose.pose.position.y + self.ekf_x
		self.y_filtered = msg.pose.pose.position.x + self.ekf_y
		self.ekfpose = msg


	def optitrackCallback(self, msg):
    		
		self.x_truth = msg.pose.position.x + self.opti_x
		self.y_truth = - msg.pose.position.y + self.opti_y
		
		# self.groundTruth = msg
		# self.poseGraph()


	def poseGraph(self):
		'''
		draw the mesearued pose
		'''
		
		plt.ion()

		print("Robot pose       = ",self.x_robot, self.y_robot)
		print("filtered pose    = ",self.x_filtered, self.y_filtered)
		print('Ground truth     = ',self.x_truth, self.y_truth)

		plt.plot(self.x_robot, self.y_robot, 'b.',
				self.x_filtered, self.y_filtered, 'r.--',
				self.x_truth, self.y_truth, 'k.:')
		# blue for robot / red for filtered / black for truth 
		
		# plt.plot(self.x_robot, self.y_robot, '.b', label = 'opti track')
		# plt.plot(self.x_filtered, self.y_filtered, '.r', label = 'EKF')
		# plt.plot(self.x_truth, self.y_truth, '.k', label = 'Ground truth')
		
		plt.axis([-0.5, 0.1, -2.0, 2.0]) # x axis from -2 to 2, y axis from 0 to 3
		
		plt.grid(True)
		plt.xlabel('x axis(m)')
		plt.ylabel('y axis(m)')
		plt.title('Straight line w constant velocity')
		# plt.legend(loc = 'upper right')
		plt.draw()
		plt.pause(0.01)


##  End of class ObstacleRepresent.


if __name__ == '__main__':

	try:
		node = PoseRepresent()
		node.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")
