import os
import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pcl2

from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, PointField
from std_msgs.msg import Header


class MonoImageMessages():

    @property
    def calibration(self):
        return self._calibration

    @calibration.setter
    def calibration(self, value):
        self._calibration = value
    
    def __init__(self, image_path, camera_pad, calibration) -> None:
        self._calibration = calibration
        self._cv_bridge = CvBridge()
        self._image_path = image_path
        self._camera_pad = camera_pad
        self._info_message = CameraInfo()
        self._info_message.header.frame_id = 'image_' + camera_pad
        self._info_message.width, self._info_message.height = tuple(self.calibration['S_rect_{}'.format(camera_pad)].tolist())
        self._info_message.distortion_model = 'plumb_bob'
        self._info_message.K = self.calibration['K_{}'.format(camera_pad)]
        self._info_message.R = self.calibration['R_rect_{}'.format(camera_pad)]
        self._info_message.D = self.calibration['D_{}'.format(camera_pad)]
        self._info_message.P = self.calibration['P_rect_{}'.format(camera_pad)]

    def get(self, filename, timestamp) -> None:
        image = MonoImageMessages._mono_image(self._image_path, filename)
        image_message = self._cv_bridge.cv2_to_imgmsg(image, encoding="mono8")
        image_message.header.frame_id = 'image_' + self._camera_pad
        image_message.header.stamp = timestamp           
        self._info_message.height, self._info_message.width = image.shape[:2]
        self._info_message.header.stamp = timestamp
        return image_message, self._info_message

    @staticmethod
    def _mono_image(path, filename):
        image_filename = os.path.join(path, filename)
        return cv2.imread(image_filename, cv2.IMREAD_GRAYSCALE)


class PointCloudMessages():

    _fields = [ PointField('x', 0,  PointField.FLOAT32, 1),
                PointField('y', 4,  PointField.FLOAT32, 1),
                PointField('z', 8,  PointField.FLOAT32, 1),
                PointField('i', 12, PointField.FLOAT32, 1)]

    _header = Header()
    
    def __init__(self) -> None:
        self._header.frame_id = 'velo_link'

    def get(self, filename, timestamp) -> None:

        self._header.stamp = rospy.Time.from_sec(
            float(datetime.strftime(timestamp, "%s.%f")))

        scan = (
            np.fromfile(
                filename, dtype=np.float32)
            ).reshape(-1, 4)

        return pcl2.create_cloud(self._header, self._fields, scan)
    