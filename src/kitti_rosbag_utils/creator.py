import sys

try:
    import pykitti
except ImportError as e:
    print('Could not load module \'pykitti\'. Please run `pip install pykitti`')
    sys.exit(1)

import os
import argparse
import rospy
import rosbag

from .writers import RawWriter, OdometryWriter

def create():

    data_types = ["raw","odometry"]
    sequences = []
    for s in range(22):
        sequences.append(str(s).zfill(2))

    parser = argparse.ArgumentParser(description = "KITTI to ROS bag conversion.")    
    parser.add_argument("-t", "--type", choices = data_types, help = "valid types: [raw,odometry].")
    parser.add_argument("-b", "--basedir", default = os.getcwd(), help = "kitti dataset base directory.")
    parser.add_argument("-s", "--sequence", choices = sequences,help = "sequence number (between 00 - 21).")
    parser.add_argument("-dt", "--date", help = "date of the raw dataset (i.e. 2011_09_26).")
    parser.add_argument("-dr", "--drive", help = "drive number of the raw dataset (i.e. 0001).")

    args = parser.parse_args()

    if args.type.find("raw") != -1:

        if args.date == None:
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -d <date> -r <drive>")
            sys.exit(1)
        
        if args.drive == None:
            print("Usage for raw dataset: kitti2bag raw_synced [dir] -d <date> -r <drive>")
            sys.exit(1)

        try:
            target_bag = rosbag.Bag(
                "kitti_raw_{}_{}.bag".format(args.date,args.drive), 'w',
                compression=rosbag.Compression.NONE)
            writer = RawWriter()
            writer.init(args.basedir, args.date, args.drive)
            writer.to_rosbag(target_bag) 
        finally:
            pass

    elif args.type.find("odometry") != -1:

        if args.sequence == None:
            print("Input sequence not given. Mandatory for odometry dataset.")
            sys.exit(1)

        try:
            target_bag = rosbag.Bag(
                "kitti_odometry_sequence_{}.bag".format(args.sequence), 'w',
                compression=rosbag.Compression.NONE)
            writer = OdometryWriter(args.basedir, args.sequence)
            writer.to_rosbag(target_bag)
        finally:
            pass

    