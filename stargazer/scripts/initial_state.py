#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import time
import numpy as np
from stargazer import StarGazer
from geometry_msgs.msg import (Point, Quaternion, Pose, PoseArray,
                               Transform, TransformStamped,
                               PoseWithCovariance, PoseWithCovarianceStamped)
from tf.transformations import euler_from_quaternion
import numpy as np


class PoseRepresent:

    def __init__(self):
        
        # ROS parameters
        # self.obstacle_topic = rospy.get_param("/obstacle_predictor/obstacle_topic")
        self.step = []
        self.count = 0
        self.timer = 0

        # Initialize ros node
        rospy.init_node('initial_pose_node', anonymous = True)
        # Subscriber
        rospy.Subscriber('marker_poses', PoseWithCovarianceStamped, self.markerCallback, queue_size=1)
        

    def spin(self):
        rospy.spin()


    def markerCallback(self, msg):
        
        self.start_t = time.time()
        self.marker = msg


    def stackPose(self):
        '''
        estimate initial pose
        '''
        _x = np.array([])
        _y = np.array([])
        _z = np.array([])
        _euler = np.array([])      
                
        while:
             
        # _x.append(self.marker.pose.pose.position.x)
        # _y.append(self.marker.pose.pose.position.y)
        # _z.append(self.marker.pose.pose.position.z)

        _x = np.append(_x, self.marker.pose.pose.position.x)
        _y = np.append(_y, self.marker.pose.pose.position.y)
        _z = np.append(_z, self.marker.pose.pose.position.z)

        _pose = np.stack([_x,_y,_z], axis=0)

        _quaternion = (self.marker.pose.pose.orientation.x,
                        self.marker.pose.pose.orientation.y,
                        self.marker.pose.pose.orientation.w,
                        self.marker.pose.pose.orientation.z)

        _roll, _pitch, _yaw = euler_from_quaternion(_quaternion)

        _euler = np.stack([

        self.averagedPose(_position, _euler)

        


    def averagedPose(self, _pose):
        '''
        average stacked pose and publish
        '''
        init_x = np.mean(_x)
        init_y = np


    def averageFunc(self, poselist):

        AVG = sum(poselist, 0.0) / len(poselist)

        return AVG


        
    def poseGraph(self):
        '''
        draw the mesearued pose
        '''
        
        # _x = []
        # _y = []
        
        plt.ion()
        
        _x = self.marker.marker_poses[0].position.x
        _y = self.marker.marker_poses[0].position.y

        # for marker in self.marker.marker_poses:
        
        #     _x.append(marker.position.x)
        #     _y.append(marker.position.x)
            

        plt.plot(_x, _y, '.b') # '.b' = blue point mark at start point
        
        plt.axis([0, 2, -1, 1]) # x axis from -2 to 2, y axis from 0 to 3
        
        plt.xlabel('horizontal(m)')
        plt.ylabel('vertical(m)')
        plt.title('map')
        plt.legend(loc = 'upper right')
        plt.draw()
        plt.pause(0.01)
        self.timer += 1
        # if self.timer % 20 == 0:
        #     plt.clf()
        print("end of the graph")
        # print("time: ", time.time() - self.start_t) # check calculated time


##  End of class ObstacleRepresent.


if __name__ == '__main__':

    try:
        node = PoseRepresent()
        node.spin()
    except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")
