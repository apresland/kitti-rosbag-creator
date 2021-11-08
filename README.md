
# Rosbags from the KITTI benchmark.

The [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/) is a popular autonomous driving dataset from Karlsruhe Institute of Technology and Toyota Technological Institute at Chicago. KITTI provides stereo, flow, scene flow, depth, odomerty, object, tracking, road, semantics and the raw data. To use the dataset for development in ROS, the streams can be converted to ROS messages and stored in a rosbag. Replaying the bag allows the KITTI data to be published over various topics.

https://user-images.githubusercontent.com/5468707/140727191-0d65cdf2-0852-4588-92cc-d740a1387cc9.mp4

https://user-images.githubusercontent.com/5468707/140726557-f6235973-dd49-4d0b-b9bd-eba0cc7b620a.mp4

## Build
python3 -m build

## Install
python3 -m pip install .

## Create
python3 -m kitti_rosbag2 -t raw -b /data/kitti/raw -dt 2011_10_03  -dr 0027

## Play
rosbag play kitti_raw_2011_10_03_0027.bag

