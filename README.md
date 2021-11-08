
# KITTI Benchmark to ROS bagfile conversion.

The [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/) is a popular autonomous driving dataset from Karlsruhe Institute of Technology and Toyota Technological Institute at Chicago. KITTI provides stereo, flow, scene flow, depth, odomerty, object, tracking, road, semantics and the raw data.

[ROS bags](http://wiki.ros.org/rosbag) are files that contain timestamped, serialized message data from ROS topics. Messages in ROS bags can be played back to aid development. In the automotive industry, ROS bag files are frequently used to capture drive data from test vehicles configured with cameras, LIDAR, GPS, and other input devices. The data for each device is stored as a topic in the ROS bag file and are played back based on their original timestamp and include the original payload. The messages from the ROS bags replace observations from the real world.

Converting KITTI data to rosbags has a number of advantages. It has a compact file based format (rather than a directory structure) that can be persisted easily in a object store (e.g. S3) for sharing by multiple team members and once created integrates directly with ROS without need for bespoke code


https://user-images.githubusercontent.com/5468707/140727191-0d65cdf2-0852-4588-92cc-d740a1387cc9.mp4

https://user-images.githubusercontent.com/5468707/140726557-f6235973-dd49-4d0b-b9bd-eba0cc7b620a.mp4

## Get some data
The KITTI benchmark is open access but you will need to register to be able to download. You can do that by following the link bellow. Once that is done you can download _raw_ or _odometry_ data and decompress to a convenient base directory. Based on the parameters `{basedir}`, `{datetime}`, `{id}` you should then have a structure similar to below where the parameter values depend on the data you downloaded.

http://www.cvlibs.net/datasets/kitti/raw_data.php


```
{basedir}
│    └── {datetime}
│        └── {datetime}_drive_{id}_sync
│            ├── image_00
│            │   └── data
│            ├── image_01
│            │   └── data
│            ├── image_02
│            │   └── data
│            ├── image_03
│            │   └── data
│            ├── oxts
│            │   └── data
│            └── velodyne_points
│                └── data
```


## Install the module

Clone this repository
```bash
git clone https://github.com/apresland/kitti-rosbag-utils
```

Build and install the python module
```bash
cd kitti-rosbag-utils
python3 -m build
python3 -m pip install
```

Create a rosbag 
```bash
python3 -m kitti_rosbag2 -t raw -b /data/kitti/raw -dt {datetime}  -dr {id}
```
Replay the KITTI data
```bash
rosbag play kitti_raw_{datetime}_{id}.bag
```

