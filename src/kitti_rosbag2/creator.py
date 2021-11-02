import sys

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)

import os
import rospy
import argparse
import cv2
import rosbag
import tf
import numpy as np
from datetime import datetime
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

class OdometryWriter:

    def __init__(self, basedir, sequence) -> None:
        self._basedir = basedir
        self._sequence = sequence

    def write(self):

        self._odometry = pykitti.odometry(self._basedir, self._sequence)

        if not os.path.exists(self._odometry.sequence_path):
            print('Path {} does not exists. Exiting.'.format(self._odometry.sequence_path))
            sys.exit(1)
             
        if len(self._odometry.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        if len(self._odometry.poses) == 0:
            print('Pose is empty? Exiting.')
            sys.exit(1)


        self._calibration = pykitti.utils.read_calib_file(os.path.join(self._basedir,'sequences', self._sequence, 'calib.txt'))
        self._bag = rosbag.Bag(
            "kitti_odometry_sequence_{}.bag".format(self._sequence), 'w',
            compression=rosbag.Compression.NONE)
        self._initial_time = (datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
        self._bridge = CvBridge()

        self.write_poses()
        self.write_mono_images(camera=0)
        self.write_mono_images(camera=1)


    def write_poses(self):
        print("Exporting time dependent transformations")

        timestamps = map(lambda x: self._initial_time + x.total_seconds(), self._odometry.timestamps)
        for timestamp, tf_matrix in zip(timestamps, self._odometry.poses):
            tf_msg = TFMessage()
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = rospy.Time.from_sec(timestamp)
            tf_stamped.header.frame_id = 'world'
            tf_stamped.child_frame_id = 'camera_left'
            
            t = tf_matrix[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(tf_matrix)
            transform = Transform()

            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]

            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped.transform = transform
            tf_msg.transforms.append(tf_stamped)

            self._bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)

    def write_mono_images(self, camera):

        camera_pad = '{0:01d}'.format(camera)
        image_path = os.path.join(self._odometry.sequence_path, 'image_{}'.format(camera_pad))
        image_filenames = sorted(os.listdir(image_path))
        timestamps = map(lambda x: self._initial_time + x.total_seconds(), self._odometry.timestamps)

        for timestamp, filename in zip(timestamps, image_filenames):

            image_filename = os.path.join(image_path, filename)
            image_data = cv2.imread(image_filename, cv2.IMREAD_GRAYSCALE)
            image_message = self._bridge.cv2_to_imgmsg(image_data, encoding="mono8")
            image_message.header.frame_id = 'image_' + camera_pad
            image_message.header.stamp = rospy.Time.from_sec(timestamp)
            self._bag.write('/camera/' + camera_pad + '/image_rect', image_message, t = image_message.header.stamp)

            info_message = CameraInfo()
            info_message.header.frame_id = 'image_' + camera_pad
            info_message.P = self._calibration['P{}'.format(camera_pad)]
            info_message.height, info_message.width = image_data.shape[:2]
            info_message.header.stamp = rospy.Time.from_sec(timestamp)
            self._bag.write('/camera/' + camera_pad + '/camera_info', info_message, info_message.header.stamp) 

def create():

    data_types = ["odometry"]
    sequences = []
    for s in range(22):
        sequences.append(str(s).zfill(2))

    parser = argparse.ArgumentParser(description = "KITTI to ROS bag conversion.")    
    parser.add_argument("-t", "--type", choices = data_types, help = "valid types: [odometry].")
    parser.add_argument("-b", "--basedir", default = os.getcwd(), help = "kitti dataset base directory.")
    parser.add_argument("-s", "--sequence", choices = sequences,help = "sequence number (between 00 - 21).")
    args = parser.parse_args()

    if args.type.find("odometry") != -1:

        if args.sequence == None:
            print("Input sequence not given. Mandatory for odometry dataset.")
            sys.exit(1)

        try:
            writer = OdometryWriter(args.basedir, args.sequence)
            writer.write()
        finally:
            pass