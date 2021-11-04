import sys
import os
import cv2
import rospy
import pykitti
import tf
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header

from sensor_msgs.msg import CameraInfo, PointField
from cv_bridge import CvBridge
from datetime import datetime
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform

from abc import ABC, abstractmethod

from .data import Dataset, RawData, OdometryData
from .transforms import StaticTransforms

class BaseWriter(ABC):

    @abstractmethod
    def _write_mono(self, target_bag, camera):

        assert isinstance(self.data, Dataset), 'Data is not a <<Dataset>> instance!'
        cv_bridge = CvBridge()

        camera_pad = '{0:02d}'.format(camera)
        dataset, image_path = self.data.dataset(camera)

        for timestamp, filename in dataset:
            image_filename = os.path.join(image_path, filename)
            image_data = cv2.imread(image_filename, cv2.IMREAD_GRAYSCALE)
            image_message = cv_bridge.cv2_to_imgmsg(image_data, encoding="mono8")
            image_message.header.frame_id = 'image_' + camera_pad
            image_message.header.stamp = timestamp           

            self._info_message.height, self._info_message.width = image_data.shape[:2]
            self._info_message.header.stamp = timestamp

            target_bag.write(
                '/camera/' + camera_pad + '/image_rect',
                image_message, t = image_message.header.stamp)

            target_bag.write(
                '/camera/' + camera_pad + '/camera_info',
                self._info_message, self._info_message.header.stamp) 



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
        camera_pad = '{0:02d}'.format(camera)
        self._info_message = CameraInfo()
        self._info_message.header.frame_id = 'image_' + camera_pad
        self._info_message.width, self._info_message.height = tuple(self.data.calibration['S_rect_{}'.format(camera_pad)].tolist())
        self._info_message.distortion_model = 'plumb_bob'
        self._info_message.K = self.data.calibration['K_{}'.format(camera_pad)]
        self._info_message.R = self.data.calibration['R_rect_{}'.format(camera_pad)]
        self._info_message.D = self.data.calibration['D_{}'.format(camera_pad)]
        self._info_message.P = self.data.calibration['P_rect_{}'.format(camera_pad)]
        super(
            RawWriter, self
            )._write_mono(target_bag, camera)

    def _write_velodyne(self, target_bag):
        print("Exporting velodyne data")
        velo_path = os.path.join(self.data.path, 'velodyne_points')
        velo_data_dir = os.path.join(velo_path, 'data')
        filenames = sorted(os.listdir(velo_data_dir))
        with open(os.path.join(velo_path, 'timestamps.txt')) as f:
            lines = f.readlines()
            velo_datetimes = []
            for line in lines:
                if len(line) == 1:
                    continue
                dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
                velo_datetimes.append(dt)

        for dt, filename in zip(velo_datetimes, filenames):
            if dt is None:
                continue

            velo_filename = os.path.join(velo_data_dir, filename)

            # read binary data
            scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

            # create header
            header = Header()
            header.frame_id = 'velo_link'
            header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))

            # fill pcl msg
            fields = [PointField('x', 0,  PointField.FLOAT32, 1),
                      PointField('y', 4,  PointField.FLOAT32, 1),
                      PointField('z', 8,  PointField.FLOAT32, 1),
                      PointField('i', 12, PointField.FLOAT32, 1)]
            pcl_msg = pcl2.create_cloud(header, fields, scan)

            target_bag.write('/velo/pointcloud', pcl_msg, t=pcl_msg.header.stamp)   


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

        if len(kitti.timestamps) == 0:
            print('Dataset is empty? Exiting.')
            sys.exit(1)

        if len(kitti.poses) == 0:
            print('Pose is empty? Exiting.')
            sys.exit(1)

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