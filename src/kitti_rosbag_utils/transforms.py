import tf
import rospy
import numpy as np

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform

class StaticTransforms:
    
    @property
    def data(self) -> None:
        return self._data

    @data.setter
    def data(self, value):
        self._data = value

    def __init__(self, data) -> None:
        self._data = data

        T_base_link_to_imu = np.eye(4, 4)
        T_base_link_to_imu[0:3, 3] = [-2.71/2.0-0.05, 0.32, 0.93]

        self._tf_msg = TFMessage()

        tfs_msg = self._create_stamped_transform(
            frame_from='base_link', frame_to='imu_link',
            transform=T_base_link_to_imu)
        self._tf_msg.transforms.append(tfs_msg)

        tfs_msg = self._create_stamped_transform(
            frame_from='imu_link', frame_to='velo_link',
            transform=self._invert(self.data.calib.T_velo_imu))
        self._tf_msg.transforms.append(tfs_msg)       

        tfs_msg = self._create_stamped_transform(
            frame_from='imu_link', frame_to='camera_0',
            transform=self._invert(self.data.calib.T_cam0_imu))
        self._tf_msg.transforms.append(tfs_msg)

        tfs_msg = self._create_stamped_transform(
            frame_from='imu_link', frame_to='camera_1', 
            transform=self._invert(self.data.calib.T_cam1_imu))
        self._tf_msg.transforms.append(tfs_msg)

        tfs_msg = self._create_stamped_transform(
            frame_from='imu_link', frame_to='camera_2',
            transform=self._invert(self.data.calib.T_cam2_imu))
        self._tf_msg.transforms.append(tfs_msg)

        tfs_msg = self._create_stamped_transform(
            frame_from='imu_link', frame_to='camera_3', 
            transform=self._invert(self.data.calib.T_cam3_imu))
        self._tf_msg.transforms.append(tfs_msg) 

    def _create_stamped_transform(self, frame_from, frame_to, transform):
        t = transform[0:3, 3]
        q = tf.transformations.quaternion_from_matrix(transform)
        tfs_msg = TransformStamped()
        tfs_msg.header.frame_id = frame_from
        tfs_msg.child_frame_id = frame_to
        tfs_msg.transform.translation.x = float(t[0])
        tfs_msg.transform.translation.y = float(t[1])
        tfs_msg.transform.translation.z = float(t[2])
        tfs_msg.transform.rotation.x = float(q[0])
        tfs_msg.transform.rotation.y = float(q[1])
        tfs_msg.transform.rotation.z = float(q[2])
        tfs_msg.transform.rotation.w = float(q[3])
        return tfs_msg

    def _invert(self, transform):
        "Invert rigid body transformation matrix"
        R = transform[0:3, 0:3]
        t = transform[0:3, 3]
        t_inv = -1 * R.T.dot(t)
        transform_inv = np.eye(4)
        transform_inv[0:3, 0:3] = R.T
        transform_inv[0:3, 3] = t_inv
        return transform_inv

    def save(self, target_bag):
        print("Exporting static transformations")
        for timestamp in self.data.timestamps:
            time = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
            for i in range(len(self._tf_msg.transforms)):
                self._tf_msg.transforms[i].header.stamp = time
            target_bag.write('/tf_static', self._tf_msg, t=time)
