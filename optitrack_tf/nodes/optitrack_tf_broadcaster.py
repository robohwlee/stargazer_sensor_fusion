#!/usr/bin/env python  
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import copy
import numpy as np
import sys


class TfForExperiment:

    def __init__(self):
        
        # Initialize ros node
        rospy.init_node('tf_broadcaster', anonymous=True)

        # Aligne coordinate of ground truth according to rosbag file
        numOfbagfile = 5 # 1, 3, 5 for v1, v3, v5 each
        self.opti_x, self.opti_y = self.align_optitrack(numOfbagfile)

        # Get the distance
        self.optitrack_d = None
        self.stargazer_d = None
        self.diff = []
        self.diff_x = []
        self.diff_y = []
        

        # Subscriber
        rospy.Subscriber('/vrpn_client_node/RigidBody/pose', PoseStamped, self.handle_optitrack_pose, queue_size=1)
        rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, self.handle_robot_pose, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.handle_odom_pose, queue_size=1)

        # Broadcaster
        self.br = tf2_ros.TransformBroadcaster()
        self.odom_pub = rospy.Publisher('odom_aligned',Odometry, queue_size=1)
        # self.robot_pose_pub = rospy.Publisher('robot_pose_aligned', PoseWithCovarianceStamped, queue_size=1)
        self.optitrack_pub = rospy.Publisher('/vrpn_client_node/RigidBody/pose_aligned', PoseStamped, queue_size=1)

        self.transform_buffer = geometry_msgs.msg.TransformStamped()


    def spin(self):
        rospy.spin()

    
    def calculate_distance(self):

        diff_x = self.optitrack_d.pose.position.x - self.stargazer_d.pose.pose.position.x
        diff_y = self.optitrack_d.pose.position.y - self.stargazer_d.pose.pose.position.y
        # print("x = ",diff_x)
        # print("y = ", diff_y)
        diff = math.sqrt(diff_x**2 + diff_y**2)

        # print("diff =",diff)
        self.diff.append(diff)
        self.diff_x.append(diff_x)
        self.diff_y.append(diff_y)
        if len(self.diff_x) > 1000:
            x = np.mean(np.array(self.diff_x))
            y = np.mean(np.array(self.diff_y))
            print("x and y = ",x,y)

        
    def align_optitrack(self, numOfdata):

        if numOfdata == 1:
            x = - 0.07
            y = 0.1
        elif numOfdata == 3:
            x = + 0.16
            y = - 0.09
        elif numOfdata == 5:
            x = - 0.54
            y = - 0.09
        else:
            sys.stdout.write("You passed wrong bag file. Please check the number again")

        return x, y


    def handle_optitrack_pose(self, msg):
        '''
        transform from optitrack frame to robot(base_link)
        '''
        self.optitrack_d = msg
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        theta = self.rad_from_degree(180)
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.optitrack_update(msg)
        self.br.sendTransform(t)


    def optitrack_update(self, msg):
        '''
        align opti track orientation value to synchronize with stargazer frame
        '''
        optitrack_msg = copy.deepcopy(msg)
        self.optitrack_d = msg

        optitrack_msg.pose.position.x = msg.pose.position.x + self.opti_x
        optitrack_msg.pose.position.y = - msg.pose.position.y + self.opti_y

        quaternion = np.array([optitrack_msg.pose.orientation.x,
                    optitrack_msg.pose.orientation.y,
                    optitrack_msg.pose.orientation.z,
                    optitrack_msg.pose.orientation.w])
        
        quaternion_aligned = self.checkFlipping(quaternion)
        
        optitrack_msg.pose.orientation.x = quaternion_aligned[0]
        optitrack_msg.pose.orientation.y = quaternion_aligned[1]
        optitrack_msg.pose.orientation.z = quaternion_aligned[2]
        optitrack_msg.pose.orientation.w = quaternion_aligned[3]

        self.optitrack_pub.publish(optitrack_msg)

    
    def handle_odom_pose(self, msg):
        '''
        transform from optitrack frame to robot(odom)
        '''
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = "odom"   # odom have child_framd as base_footprint
        t.transform.translation.x = 0.00
        t.transform.translation.y = 5.0
        t.transform.translation.z = 0.0
        theta = self.rad_from_degree(-90)
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.odom_update(msg)
        self.br.sendTransform(t)

        
    def odom_update(self, msg):
        '''
        align odom twist value from x to y to synchronize with stargazer frame
        '''
        odom_msg = copy.deepcopy(msg)
        odom_msg.pose.pose.position.x = msg.pose.pose.position.y # foward at stargazer frame
        odom_msg.pose.pose.position.y = msg.pose.pose.position.x
        odom_msg.twist.twist.linear.x = msg.twist.twist.linear.y # foward at stargazer frame
        odom_msg.twist.twist.linear.y = msg.twist.twist.linear.x

        self.odom_pub.publish(odom_msg)


    def handle_robot_pose(self, msg):
        '''
        synchronize stargazer's frame of global(map) and local center(stragazer)
        '''
        self.stargazer_d = msg
        self.calculate_distance()

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "stargazer"
        t.child_frame_id = "map"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0.00
        theta = self.rad_from_degree(0)
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)


    def rad_from_degree(self, degree):

        return degree * math.pi / 180


    def checkFlipping(self, quaternion):
        '''
        prevent quaternion from filping
        '''
        quaternion_inverse = np.array([-quaternion[0],-quaternion[1],-quaternion[2],quaternion[3]])
        dot_quat = np.dot(quaternion,quaternion_inverse)
        
        # if dot_quat < 0:
        #     quaternion_aligned = quaternion_inverse
        # else:
        #     quaternion_aligned = quaternion
        
        if quaternion[3] < 0:
            quaternion_aligned = - quaternion
        else:
            quaternion_aligned = quaternion

        return quaternion_aligned

    # sample function for dynamic transformation
    # def handle_optitrack_pose(self, msg):
        
    #     t = self.transform_buffer
    #     t.header.stamp = msg.header.stamp
    #     t.header.frame_id = "world"
    #     t.child_frame_id = "base_link"
    #     t.transform.translation.x = msg.pose.position.x
    #     t.transform.translation.y = msg.pose.position.y
    #     t.transform.translation.z = 0.0
    #     # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    #     t.transform.rotation.x = msg.pose.orientation.x
    #     t.transform.rotation.y = msg.pose.orientation.y
    #     t.transform.rotation.z = msg.pose.orientation.z
    #     t.transform.rotation.w = msg.pose.orientation.w

    #     self.br.sendTransform(t)


if __name__ == '__main__':

    try:
        node = TfForExperiment()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")