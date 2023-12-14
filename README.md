
# KITTI Benchmark to ROS bagfile.

The [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/) is a popular autonomous driving dataset from Karlsruhe Institute of Technology and Toyota Technological Institute at Chicago. KITTI provides stereo, flow, scene flow, depth, odomerty, object, tracking, road, semantics and the raw data.


## ROS bagfiles
[ROS bags](http://wiki.ros.org/rosbag) are files that contain timestamped, serialized message data from ROS topics. Messages in ROS bags can be played back to aid development. In the automotive industry, ROS bag files are frequently used to capture drive data from test vehicles configured with cameras, LIDAR, GPS, and other input devices. The data for each device is stored as a topic in the ROS bag file and are played back based on their original timestamp and include the original payload. The messages from the ROS bags replace observations from the real world.


## Why do I need this?
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

### Get The Code

Clone this repository
```bash
git clone https://github.com/apresland/kitti-rosbag-utils
```

### Create a Virtual Environment (optional but recommended)

Once the code has been downloaded or cloned to a folder, it is recommended (but not required) to create a Python virtual environment. 
Making a virtual environment will allow you to install the project in it's own environment instead of your computer's global environment.
If you decide not to create a virtual environment, please skip to the 'Build and Install' section. Otherwise, continue following the instructions in this section. 

Open a terminal in the project folder and run this command (replacing `python3` with `python` as necessary for your installation):

```
python3 -m venv venv
```

#### Activating The Virtual Environment

Next, you will need to activate the virtual environment. To activate the virtual environment on a UNIX style operating system (Linux, macOS, etc.), run this command:

```
source venv/bin/activate
```

To activate the virtual environment in Windows, run this command from the project folder root:

```
.\venv\Scripts\activate
```

You will need to have this virtual environment activated in order to run the code in this project. If your virtual environment ever becomes deactivated, you may run the above command (for your 
applicable OS) again from the project folder root. 

#### Deactivating The Virtual Environment

When you're done playing with this program, you can deactivate the virtual environment.

On Linux, run:

```
deactivate
```

On Windows, in the root project folder run:

```
.\venv\Scripts\deactivate
```

You can reactivate the virtual environment when you feel like using the program again by running the commands 
mentioned above for your specific operating system. FYI, you only need to create the virtual environment once. 
You can activate it and deactivate it as many times as you need to. 

### Build and Install

First, make sure the build python module is installed by running

```bash
python3 -m pip install build
```

Then, from inside the project folder root you can run

```bash
python3 -m build
python3 -m pip install
```

## Create a rosbag 
Create a rosbag from raw KITTI data using the following command
```bash
python3 -m kitti_rosbag_utils -t raw -b {basedir} -dt {datetime}  -dr {id}
```
## Replay the data
Replay the data with the rosbag CLI (install it first)
```bash
rosbag play kitti_raw_{datetime}_{id}.bag
```
View the data in RViz with the provided config
```bash
rviz -d kitti.rviz
```
