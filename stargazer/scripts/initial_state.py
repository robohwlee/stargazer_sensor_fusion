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
from auv_msgs.msg import RPY
import numpy as np


class InitPoseCalculator:

    def __init__(self):
        
        # ROS parameters
        self.stargazer_frame_id = rospy.get_param('~stargazer_frame_id', 'stargazer')
        self.map_frame_prefix = rospy.get_param('~map_frame_prefix', 'stargazer/map_')
        self.marker_frame_prefix = rospy.get_param('~marker_frame_prefix', 'stargazer/marker_')
        self._x = np.array([])
        self._y = np.array([])
        self._angle = np.array([])
        self.pose_count = 0
        self.count_limit = 50

        # Initialize ros node
        rospy.init_node('initial_pose_node', anonymous = True)
        # Subscriber
        rospy.Subscriber('marker_poses', PoseWithCovarianceStamped, self.markerCallback, queue_size=1)
        # Publisher
        self.init_pose_pub = rospy.Publisher('init_pose', InitPose, queue_size=1)


    def spin(self):
        rospy.spin()


    def markerCallback(self, msg):
        
        self.marker = msg
        self.getPose()


    def getPose(self):
        '''
        estimate initial pose
        '''
        self._x = np.append(self._x, self.marker.pose.pose.position.x)
        self._y = np.append(self._y, self.marker.pose.pose.position.y)
        
        _quaternion = (self.marker.pose.pose.orientation.x,
                    self.marker.pose.pose.orientation.y,
                    self.marker.pose.pose.orientation.w,
                    self.marker.pose.pose.orientation.z)
        _roll, _pitch, _yaw = euler_from_quaternion(_quaternion)
        self._angle = np.append(_angle, _yaw)

        self.pose_count += 1
        
        # _position = np.vstack([_x,_y])
        if self.pose_count > self.count_limit:
            _pose = np.stack([_x,_y,_angle], axis=0)
            self.averagedPose(_pose)

       
    def averagedPose(self, _pose):
        '''
        average stacked pose and publish
        '''
        stamp = rospy.Time.now()
        init_pose_msg = InitPose()
        init_pose_msg.header.stamp = stamp
        init_pose_msg.header.frame_id = stargazer
        init_pose_msg.position.x = np.mean(_pose[0, 10:40])
        init_pose_msg.position.y = np.mean(_pose[1, 10:40])
        init_pose_msg.euler.yaw = np.mean(_pose[2, 10:40])
        
        self.init_pose_pub.publish(init_pose_msg)



if __name__ == '__main__':

    try:
        node = InitPoseCalculator()
        node.spin()
    except rospy.ROSInterruptException:
		rospy.loginfo("node terminated.")
