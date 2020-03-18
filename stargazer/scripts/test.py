#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import time
import numpy as np
from stargazer import StarGazer
from geometry_msgs.msg import PoseWithCovarianceStamped
from stargazer.msg import (MarkerPose, MarkerPoses,
                           MarkerRawPose, MarkerRawPoses)

class PoseRepresent:

    def __init__(self):
        
        # ROS parameters
        rospy.set_param('vrpn_client_node/RigidBody/pose', 'optitrack')
        rospy.set_param('step', [])
        rospy.set_param('count', 0)
        rospy.set_param('timer', 0)

        # Initialize ros node
        rospy.init_node('pose_represent', anonymous = True)
        # Subscriber
        rospy.Subscriber('marker_poses', PoseWithCovarianceStamped, self.markerCallback, queue_size=1)    # local pose
        rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, self.robotCallback, queue_size=1)    # global pose
        rospy.Subscriber('optitrack', PoseStamped, self.groundCallback, queue_size=1)    # global pose
        

    def spin(self):
        rospy.spin()


    def markerCallback(self, msg):
        
        self.markerpose = msg


    def robotCallback(self, msg):
        
        self.robotpose = msg
        self.poseGraph()


    def groundCallback(self, msg):
        
        self.groundTruth = msg
        self.poseGraph()


    def poseGraph(self):
        '''
        draw the mesearued pose
        '''
        
        plt.ion()
        
        x_robot = self.robotpose.pose.pose.position.x
        y_robot = self.robotpose.pose.pose.position.y

        x_truth = self.groundTruth.pose.position.x
        y_truth = self.groundTruth.pose.position.y

        print("Robot pose   = ",x_robot, y_robot)
        print("Ground truth = ",x_truth, y_truth)

        plt.plot(x_robot, y_robot, '.b') # '.b' = blue point mark at start point
        plt.plot(x_truth, y_truth, '.r') # '.b' = red point mark at start point
        
        plt.axis([-1, 2, 0, 2]) # x axis from -2 to 2, y axis from 0 to 3
        
        plt.xlabel('horizontal(m)')
        plt.ylabel('vertical(m)')
        plt.title('map')
        plt.legend(loc = 'upper right')
        plt.draw()
        plt.pause(0.01)
        self.timer += 1
        # if self.timer % 20 == 0:
        #     plt.clf()
        # print("end of the graph")
        # print("time: ", time.time() - self.start_t) # check calculated time


##  End of class ObstacleRepresent.


if __name__ == '__main__':

    try:
        node = PoseRepresent()
        node.spin()
    except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")
