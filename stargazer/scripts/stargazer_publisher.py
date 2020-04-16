#!/usr/bin/env python
import rospy
import numpy
import tf
from stargazer import StarGazer
from std_msgs.msg import String, Int16
from geometry_msgs.msg import (Point, Quaternion, Pose, PoseArray,
                               Transform, TransformStamped,
                               PoseWithCovariance, PoseWithCovarianceStamped)
from tf2_msgs.msg import TFMessage
from stargazer.msg import (MarkerPose, MarkerPoses,
                           MarkerRawPose, MarkerRawPoses)


def tf_to_matrix(trans, rot):
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)
    return numpy.dot(trans_mat, rot_mat)

def matrix_to_tf(mat):
    trans = tf.transformations.translation_from_matrix(mat)
    rot = tf.transformations.quaternion_from_matrix(mat)
    return trans, rot

def matrix_to_pose(mat):
    trans, rot = matrix_to_tf(mat)
    return Pose(
        position=Point(*trans),
        orientation=Quaternion(*rot)
    )

def matrix_to_transform(mat):
    trans, rot = matrix_to_tf(mat)
    return Transform(
        translation=Point(*trans),
        rotation=Quaternion(*rot)
    )

class StarGazerNode(object):
    def __init__(self):
        self.marker_map = rospy.get_param('~marker_map', {})
        self.fixed_frame_id = rospy.get_param('~fixed_frame_id', 'map')
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'base_link')
        self.stargazer_frame_id = rospy.get_param('~stargazer_frame_id', 'stargazer')
        self.map_frame_prefix = rospy.get_param('~map_frame_prefix', 'stargazer/map_')
        self.marker_frame_prefix = rospy.get_param('~marker_frame_prefix', 'stargazer/marker_')
        self.covariance = rospy.get_param('~covariance', [0.01]*36)
        if len(self.covariance) != 36:
            raise Exception('The "covariance" parameter must be a 36 element vector.')

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher('robot_pose', PoseWithCovarianceStamped, queue_size=1)
        self.pose_array_pub = rospy.Publisher('robot_pose_array', PoseArray, queue_size=1)
        self.marker_poses_raw_pub = rospy.Publisher('marker_raw_poses', MarkerRawPoses, queue_size=1)
        self.marker_poses_pub = rospy.Publisher('marker_poses', PoseWithCovarianceStamped, queue_size=1)
        # self.marker_poses_pub = rospy.Publisher('marker_poses', MarkerPoses, queue_size=1)
        self.eventin_sub = rospy.Subscriber("~event_in", String, self.callback_set_param)
        self.eventout_pub = rospy.Publisher("~event_out", String, queue_size=1)
        self.rawresponse_pub = rospy.Publisher("~raw_response", String, queue_size=1)
        self.map_marker_sub = rospy.Subscriber("map_marker", Int16, self.callback_map_marker)
        self.unknown_ids = set()

    def run(self):
        # Publish static TF frames for the Stargazer map.
        stamp_now = rospy.Time.now()
        map_tf_msg = TFMessage()

        # make an transform msg for each marker using marker_map
        # marker_map: dictionary of marker transforms, {marker_id: (4,4) matrix}  , (i.e. Tmap_makrer = (4,4) matrix)
        for marker_id, Tmap_marker in self.marker_map.iteritems():
            marker_tf_msg = TransformStamped()
            marker_tf_msg.header.stamp = stamp_now
            marker_tf_msg.header.frame_id = self.fixed_frame_id # map
            marker_tf_msg.child_frame_id = self.map_frame_prefix + marker_id    # stargazer/map_ + (marker_id)
            marker_tf_msg.transform = matrix_to_transform(Tmap_marker)
            map_tf_msg.transforms.append(marker_tf_msg)

        self.tf_static_pub = rospy.Publisher('tf_static', TFMessage, latch=True, queue_size=1)
        self.tf_static_pub.publish(map_tf_msg)

        stargazer_args = {
            'device': rospy.get_param('~device_port', ''),
            'marker_map': self.marker_map,
            'callback_global': self.callback_global,
            'callback_local': self.callback_local,
            'callback_raw': self.callback_raw,
            'callback_raw_reponse': self.callback_raw_reponse,
        }

        # Start publishing Stargazer data.
        with StarGazer(**stargazer_args) as self.stargazer:
            # Set all parameters, possibly to their default values. This defaults to True
            # as it is the safest option because the parameters can be corrupted when the
            # StarGazer is powered off.
            if rospy.get_param('~set_parameters', True):
                # The StarGazer might be streaming data. Turn off streaming mode.
                self.stargazer.stop_streaming()

                parameters = self.get_options()
                for name, value in parameters.iteritems():
                    self.stargazer.set_parameter(name, value)
                self.stargazer._send_command('SetEnd')

            # Start streaming. ROS messages will be published in callbacks.
            self.stargazer.start_streaming()

            rospy.loginfo('Stargazer Publisher node is running..')
            rospy.spin()

            # Stop streaming. Try to clean up after ourselves.
            self.stargazer.stop_streaming()


    def callback_set_param(self, msg):
        event_out_msg = 'e_failed'
        if msg.data == "e_start_stream":
            success = self.stargazer.start_streaming()
            if success:
                event_out_msg = 'e_streaming_started'

        elif msg.data == "e_stop_stream":
            success = self.stargazer.stop_streaming()
            if success:
                event_out_msg = 'e_streaming_stopped'
            else:
                success = self.stargazer.disconnect()
                success = self.stargazer.connect()
                success = self.stargazer.stop_streaming()

            if success:
                event_out_msg = 'e_streaming_stopped'

        elif 'e_update' in msg.data:
            success = self.stargazer.stop_streaming()
            event_out_msg = self.update_parameters(msg.data)

        elif msg.data == "e_connect":
            rospy.loginfo('Reconnecting stargazer..')
            success = self.stargazer.connect()
            if success:
                event_out_msg = 'e_connected'

        elif msg.data == "e_disconnect":
            rospy.loginfo('Disconnecting stargazer..')
            success = self.stargazer.disconnect()
            if success:
                event_out_msg = 'e_disconnected'
        else:
            rospy.logwarn('Command is not in list.')
        self.eventout_pub.publish(event_out_msg)

    def update_parameters(self, msg):
        event_out_msg = 'e_failed'
        success = False
        name_field = msg.split('_')[2]
        if(name_field == 'param'):
            parameters = self.get_options()
            self.stargazer.stop_streaming()
            for name, value in parameters.iteritems():
                success = self.stargazer.set_parameter(name, value)
                if not(success):
                    break
        else:
            name = name_field
            value = rospy.get_param('~'+name)
            success = self.stargazer.set_parameter(name, value)

        success = self.stargazer.set_parameter('SetEnd', '')

        if success:
            event_out_msg = 'e_parameter_updated'
        return event_out_msg

    def callback_raw_reponse(self, raw_response_msg):
        #publishing raw response of the stargazer
        msg = String()
        msg.data = str(raw_response_msg)
        self.rawresponse_pub.publish(msg)

    def callback_raw(self, pose_dict):
        
        # print("callback_raw is launched")
        # test to check which callback fucntion is launched
        
        marker_raw_poses_msg = MarkerRawPoses()
        marker_raw_poses_msg.header.frame_id = 'stargazer_raw'
        
        for pose in pose_dict:
            marker_raw_pose = MarkerRawPose()
            marker_raw_pose.marker_id.data = pose[0]
            marker_raw_pose.position.x = pose[1]
            marker_raw_pose.position.y = pose[2]
            marker_raw_pose.position.z = pose[3]
            marker_raw_pose.orientation.z = pose[4]
            marker_raw_poses_msg.marker_poses.append(marker_raw_pose)

        self.marker_poses_raw_pub.publish(marker_raw_poses_msg)


    def callback_global(self, pose_dict, unknown_ids):
        
        # print("callback_global is launched")
        # test to check which callback fucntion is launched
        
        stamp = rospy.Time.now()

        # Print a warning about unmapped IDs.
        for unknown_id in unknown_ids - self.unknown_ids:
            rospy.logwarn('Detected marker ID %s that is not in the map.', unknown_id)
            self.unknown_ids.add(unknown_id)

        # Find the transform from the Stargazer to the robot.
        try:
            Tstargazer_robot = tf_to_matrix(
                *self.tf_listener.lookupTransform(
                    self.stargazer_frame_id, self.robot_frame_id, stamp)
            )   # from robot frame to stargazer frame (i.e. from base_link to stargazer)
        except tf.Exception as e:
            rospy.logwarn('Failed looking up transform from "%s" to "%s": %s.',
                self.stargazer_frame_id, self.robot_frame_id, str(e))
            return

        # Publish the poses as ROS messages. Also publish an array of the robot
        # poses predicted from each marker. This is useful for visualization.
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = stamp
        pose_array_msg.header.frame_id = self.fixed_frame_id
        # print("in callback global pose_dict=", pose_dict)
        
        for marker_id, Tmap_stargazer in pose_dict.iteritems():
            # Convert the 'map -> stargazer' transform into a 'map -> robot' pose.
            # print("Tmap_stargazer in callback = ", Tmap_stargazer)
            Tmap_robot = numpy.dot(Tmap_stargazer, Tstargazer_robot)
            pose_msg = matrix_to_pose(Tmap_robot)
            pose_array_msg.poses.append(pose_msg)

            # Publish the output to a ROS message.
            pose_cov_msg = PoseWithCovarianceStamped()
            pose_cov_msg.header.stamp = stamp
            pose_cov_msg.header.frame_id = self.fixed_frame_id
            pose_cov_msg.pose.pose = pose_msg
            pose_cov_msg.pose.covariance = self.covariance
            self.pose_pub.publish(pose_cov_msg)

        self.pose_array_pub.publish(pose_array_msg)


    # 02.21 change the msg type from MarkerPoses to PoseWithCovarianceStamped
    # this is for single ID which got only one land mark(pose_dict)
    def callback_local(self, pose_dict):

        # print("callback_local is launched")
        # test to check which callback fucntion is launched

        stamp = rospy.Time.now()
        marker_poses_msg = PoseWithCovarianceStamped()
        marker_poses_msg.header.stamp = stamp
        # marker_poses_msg.header.frame_id = self.stargazer_frame_id  # = stargazer
        # print("pose_dict=", pose_dict)
        for marker_id, pose in pose_dict.iteritems():
            # print("pose=",pose)
            cartesian = pose[0:3, 3]
            # print("cartesian",cartesian)
            quaternion = tf.transformations.quaternion_from_matrix(pose)
            pos=Point()
            pos.x = cartesian[0]
            pos.y = cartesian[1]
            pos.z = cartesian[2]
            quat=Quaternion()
            quat.x = quaternion[0]
            quat.y = quaternion[1]
            quat.z = quaternion[2]
            quat.w = quaternion[3]
            # print("Detected marker id is", marker_id)

            # marker_poses_msg.header.frame_id = marker_id
            marker_poses_msg.pose.pose.position = pos
            marker_poses_msg.pose.pose.orientation = quat
            marker_poses_msg.header.frame_id = marker_id

            frame_id = '{:s}{:s}'.format(self.marker_frame_prefix, marker_id)

            self.tf_broadcaster.sendTransform(
                cartesian, quaternion, stamp, frame_id, self.stargazer_frame_id
            )
            # translation, rotation, time, child, parent 
        if 
        self.tf_broadcaster.sendTransform(
                cartesian, quaternion, stamp, frame_id, self.stargazer_frame_id
            )
        # specify covariance as it is required 
        # marker_poses_msg.pose.covariance = 


        self.marker_poses_pub.publish(marker_poses_msg)


    # def callback_local(self, pose_dict):
        
    #     print("callback_local is launched")
    #     # test to check which callback fucntion is launched

    #     stamp = rospy.Time.now()
    #     marker_poses_msg = MarkerPoses()
    #     marker_poses_msg.header.frame_id = self.stargazer_frame_id  # = stargazer

    #     for marker_id, pose in pose_dict.iteritems():
    #         cartesian = pose[0:3, 3]
    #         quaternion = tf.transformations.quaternion_from_matrix(pose)
    #         pos=Point()
    #         pos.x = cartesian[0]
    #         pos.y = cartesian[1]
    #         pos.z = cartesian[2]
    #         quat=Quaternion()
    #         quat.x = quaternion[0]
    #         quat.y = quaternion[1]
    #         quat.z = quaternion[2]
    #         quat.w = quaternion[3]

    #         marker_pose = MarkerRawPose()
    #         marker_pose.marker_id.data = marker_id
    #         marker_pose.position = pos
    #         marker_pose.orientation = quat
    #         marker_poses_msg.marker_poses.append(marker_pose)


    #         frame_id = '{:s}{:s}'.format(self.marker_frame_prefix, marker_id)
    #         self.tf_broadcaster.sendTransform(
    #             cartesian, quaternion, stamp, frame_id, self.stargazer_frame_id
    #         )

    #     self.marker_poses_pub.publish(marker_poses_msg)


    def callback_map_marker(self, msg):
        stamp = rospy.Time(0.0)
        print("cb map_marker launched")
        # Find the transform from the Stargazer to the robot.
        try:
            marker_id = str(msg.data)
            print("in callback mapmarker, marker_id = ",marker_id)
            marker_frame_id = '{:s}{:s}'.format(self.marker_frame_prefix, marker_id)
            Tmarker_fixed = tf_to_matrix(*self.tf_listener.lookupTransform(self.fixed_frame_id, marker_frame_id, stamp))
            self.marker_map[marker_id] = Tmarker_fixed
            print("in callback mapmarker, marker_map = ",self.marker_map[marker_id])
            print str(self.marker_map)
        except tf.Exception as e:
            rospy.logwarn('Failed looking up transform from "%s" to "%s": %s.',
                          marker_frame_id, self.fixed_frame_id, str(e))

    def get_options(self):
        """ Gets StarGazer options from the ROS parameter server.
        """
        options = {}

        # Threshold level to reject external turbulence shown in image; depend
        # on surroundings. Recommended value is ranging from 210 to 240.
        options['ThrVal'] = rospy.get_param('~ThrVal', 210)

        # Distance from a StarGazer to a landmark; used when wanting to input
        # manually the height (in millimeters).
        options['MarkHeight'] = rospy.get_param('~MarkHeight', 1847)

        # A total number of landmarks to be assigned under Map Mode.
        options['IDNum'] = rospy.get_param('~IDNum', 3)
        assert 0 <= options['IDNum'] < 4095

        # The number of reference ID under map mode.
        options['RefID'] = rospy.get_param('~RefID', 2916)

        # To determine how to get ThrVal; There are Auto and Manual. 'Manual'
        # should be assigned to use input data and 'Auto' be assigned to use a
        # value calculated automatically in StarGazer
        options['ThrAlg'] = rospy.get_param('~ThrAlg', 'Manual')
        assert options['ThrAlg'] in ['Auto', 'Manual']

        # To set up landmark type by use. There are Home and Office. Home means
        # HLDn-S landmark (up to 31 IDs) and Office means HLDn-L (up to 4095
        # IDs).
        options['MarkType'] = rospy.get_param('~MarkType', 'Office')
        assert options['MarkType'] in ['Home', 'Office']

        # To setup landmark type by height. There are different landmark types
        # for height - HLDn-2 for MarkDim 1, HLDn-3 for MarkDim 2, and HLDn-4
        # for MarkDim 3.
        options['MarkDim'] = rospy.get_param('~MarkDim', 1)
        assert options['MarkDim'] in [1, 2, 3]

        # To determine whether map building is executed or not. There are Start
        # and Stop. If action under Map Mode is required, you should set the
        # parameter to 'Start' and start Map Building.
        options['MapMode'] = rospy.get_param('~MapMode', 'Start')
        assert options['MapMode'] in ['Start', 'Stop']

        # To determine whether landmarks are used independently under Alone
        # Mode or not (dependently under Map Mode). There are Alone and Map.
        options['MarkMode'] = rospy.get_param('~MarkMode', 'Map')
        assert options['MarkMode'] in ['Alone', 'Map']

        return options

if __name__ == '__main__':
    rospy.init_node('stargazer_publisher')
    node = StarGazerNode()
    node.run()
