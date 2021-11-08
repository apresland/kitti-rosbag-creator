import sys
import os
import rospy
import pykitti
import tf

from sensor_msgs.msg import CameraInfo
from datetime import datetime
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform

from abc import ABC, abstractmethod

from .data import Dataset, RawData, OdometryData
from .messages import MonoImageMessages, PointCloudMessages
from .transforms import StaticTransforms

class BaseWriter(ABC):

    @abstractmethod
    def _write_mono(self, target_bag, camera):

        assert isinstance(self.data, Dataset), 'Data is not a <<Dataset>> instance!'

        camera_pad = '{0:02d}'.format(camera)
        dataset, image_path = self.data.dataset(camera)
        messages = MonoImageMessages(image_path, camera_pad, self.data.calibration)

        for timestamp, filename in dataset:

            image_message, info_message = messages.get(
                filename, timestamp)
                
            target_bag.write(
                '/camera/' + camera_pad + '/image_rect',
                image_message, t = image_message.header.stamp)

            target_bag.write(
                '/camera/' + camera_pad + '/camera_info',
                info_message, info_message.header.stamp) 


class RawWriter(BaseWriter):

    @property
    def data(self) -> RawData:
        return self._data

    @data.setter
    def data(self, value):
        self._data = value

    @property
    def transforms(self) -> StaticTransforms:
        return self._transforms

    @transforms.setter
    def transforms(self, value):
        self._transforms = value


    def __init__(self) -> None:
        super(RawWriter, self).__init__()


    def init(self,  basedir, date, drive) -> None:
        kitti = pykitti.raw(
            basedir, date, drive)

        if not os.path.exists(kitti.data_path):
            print('Path {} does not exists. Exiting.'.format(kitti.data_path))
            sys.exit(1)

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        kittti_calibration = pykitti.utils.read_calib_file(
            os.path.join(kitti.calib_path, 'calib_cam_to_cam.txt'))

        self.data = RawData()
        self.data.path = kitti.data_path
        self.data.calibration = kittti_calibration
        self.data.timestamps = kitti.timestamps
        self.data.start = (
            datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()

        self.transforms = StaticTransforms(kitti)


    def to_rosbag(self, target_bag):

        self.transforms.save(
            target_bag)

        self._write_mono(
            target_bag,
            camera=0)
        self._write_velodyne(
            target_bag)

    def _write_mono(self, target_bag, camera):
        super(
            RawWriter, self
            )._write_mono(target_bag, camera)

    def _write_velodyne(self, target_bag):
        print("Exporting velodyne data")
        messages = PointCloudMessages()

        for timestamp, filename in self.data.velodyne():
            if timestamp is None:
                continue
            
            pcl_msg = messages.get(
                filename, timestamp)

            target_bag.write(
                '/velo/pointcloud', 
                pcl_msg, t=pcl_msg.header.stamp)   


class OdometryWriter(BaseWriter):

    @property
    def data(self) -> OdometryData:
        return self.get("_data")

    @data.setter
    def data(self, value):
        self["_data"] = value

    def __init__(self, basedir, sequence) -> None:
        super(OdometryWriter, self).__init__()

        kitti = pykitti.odometry(
            basedir, sequence)
        
        if not os.path.exists(kitti.sequence_path):
            print('Path {} does not exists. Exiting.'
                .format(self._data.sequence_path))
            sys.exit(1)

        elif len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        elif len(kitti.poses) == 0:
            print('Pose is empty? Exiting.')
            sys.exit(1)

        else:
            self.data = OdometryData()
            self.data.path = kitti.sequence_path
            self.data.calibration = pykitti.utils.read_calib_file(
                os.path.join(basedir,
                'sequences', sequence, 'calib.txt'))
            self.data.timestamps = kitti.timestamps
            self.data.start = (
                datetime.utcnow() - datetime(1970, 1, 1)).total_seconds()
            self.data.poses = kitti.poses

    def to_rosbag(self, target_bag):
        self._write_poses(
            target_bag)
        self._write_mono(
            target_bag,
            camera=0)
        self._write_mono(
            target_bag, 
            camera=1)

    def _write_poses(self, target_bag):
        print("Exporting time dependent transformations")
        for timestamp, tf_matrix in zip(self.data.timestamps, self.data.poses):
            
            t = tf_matrix[0:3, 3]
            q = tf.transformations.quaternion_from_matrix(tf_matrix)
            s = self._initial_time + timestamp.total_seconds()

            transform = Transform()
            transform.translation.x = t[0]
            transform.translation.y = t[1]
            transform.translation.z = t[2]
            transform.rotation.x = q[0]
            transform.rotation.y = q[1]
            transform.rotation.z = q[2]
            transform.rotation.w = q[3]

            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = rospy.Time.from_sec(s)
            tf_stamped.header.frame_id = 'world'
            tf_stamped.child_frame_id = 'camera_left'
            tf_stamped.transform = transform

            tf_msg = TFMessage()
            tf_msg.transforms.append(tf_stamped)

            target_bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)


    def _write_mono(self, target_bag, camera):
        camera_pad = '{0:02d}'.format(camera)
        self._info_message = CameraInfo()
        self._info_message.header.frame_id = 'image_' + camera_pad
        self._info_message.P = self._calibration['P{}'.format(camera_pad)]
        super(
            OdometryWriter, self
            )._write_mono(target_bag, camera) 