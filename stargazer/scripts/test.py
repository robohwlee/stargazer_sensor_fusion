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
        # self.obstacle_topic = rospy.get_param("/obstacle_predictor/obstacle_topic")
        self.step = []
        self.count = 0
        self.timer = 0

        # Initialize ros node
        rospy.init_node('pose_represent', anonymous = True)
        # Subscriber
        # rospy.Subscriber('marker_poses', MarkerPoses, self.markerCallback, queue_size=1)
        # rospy.Subscriber('marker_poses', PoseWithCovarianceStamped, self.markerCallback, queue_size=1)    # local pose
        rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, self.markerCallback, queue_size=1)    # global pose
        

    def spin(self):
        rospy.spin()


    def markerCallback(self, msg):
        
        self.start_t = time.time()
        self.marker = msg
        self.poseGraph()


    def poseGraph(self):
        '''
        draw the mesearued pose
        '''
        
        # _x = []
        # _y = []
        
        plt.ion()
        
        _x = self.marker.pose.pose.position.x
        _y = self.marker.pose.pose.position.y

        # for marker in self.marker.marker_poses:
        
        #     _x.append(marker.position.x)
        #     _y.append(marker.position.x)
        print(_x,_y)

        plt.plot(_x, _y, '.b') # '.b' = blue point mark at start point
        
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
