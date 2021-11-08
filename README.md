
# Create rosbags of kitti data to replay in ROS2

## Build
python3 -m build

## Install
python3 -m pip install .

## Create
python3 -m kitti_rosbag2 -t raw -b /data/kitti/raw -dt 2011_10_03  -dr 0027

## Play
rosbag play kitti_raw_2011_10_03_0027.bag

