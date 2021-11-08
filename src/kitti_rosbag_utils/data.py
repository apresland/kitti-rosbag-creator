import os
import rospy

from abc import ABC, abstractmethod
from collections.abc import MutableMapping
from datetime import datetime

class Dataset(ABC, MutableMapping):

    def __init__(self, *args, **kwargs):
        self.__dict__.update(*args, **kwargs)

    def __setitem__(self, key, value):
        self.__dict__[key] = value

    def __getitem__(self, key):
        return self.__dict__[key]

    def __delitem__(self, key):
        del self.__dict__[key]

    def __iter__(self):
        return iter(self.__dict__)

    def __len__(self):
        return len(self.__dict__)

    @abstractmethod
    def dataset(self, camera):
        pass

    @property
    def calibration(self) -> None:
        return self.get("_calibration")

    @calibration.setter
    def calibration(self, value):
        self["_calibration"] = value

    @property
    def timestamps(self) -> None:
        return self.get("_timestamps")

    @timestamps.setter
    def timestamps(self, value):
        self["_timestamps"] = value

    @property
    def path(self) -> None:
        return self.get("_path")

    @path.setter
    def path(self, value):
        self["_path"] = value

    @property
    def start(self) -> None:
        return self.get("_start")

    @start.setter
    def start(self, value):
        self["_start"] = value

    def _zip(self, datetimes, filenames) -> None:
        return zip(datetimes, filenames)


class RawData(Dataset):
    
    def dataset(self, camera):
        camera_pad = '{0:02d}'.format(camera)
        image_dir = os.path.join(self.path, 'image_{}'.format(camera_pad))
        image_path = os.path.join(image_dir, 'data')
        filenames = sorted(os.listdir(image_path))
        datetimes = map(lambda x: rospy.Time.from_sec(float(datetime.strftime(x, "%s.%f"))), self.timestamps)
        return super(
            RawData, self)._zip(datetimes, filenames
            ), image_path

    def velodyne(self):
        velo_path = os.path.join(self.path, 'velodyne_points')
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

        velo_filenames = map(lambda x: os.path.join(velo_data_dir, x), filenames)

        return zip(velo_datetimes, velo_filenames)


class OdometryData(Dataset):

    @property
    def poses(self) -> None:
        return self.get("_poses")

    @poses.setter
    def poses(self, value):
        self["_poses"] = value

    def dataset(self, camera):
        camera_pad = '{0:01d}'.format(camera)
        image_path = os.path.join(self.path, 'image_{}'.format(camera_pad))
        filenames = sorted(os.listdir(image_path))
        datetimes = map(lambda x: rospy.Time.from_sec(self.start + x.total_seconds()), self.timestamps)
        return super(
            OdometryData, self)._zip(datetimes, filenames
            ), image_path