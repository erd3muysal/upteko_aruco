# -*- coding: utf-8 -*-
# @author: R. Erdem Uysal

import pandas as pd
import argparse
import rospy
import rosbag
from mavros_msgs.msg import Altitude
from std_msgs.msg import Int32, Float64, String

TOPIC = '/mavros/altitude'

def csv2rosbag(csvPath, outputBag):
    """
    Creates a ROS bag file from a CSV file.
    In this script, only the time and altitude informations written into the ROS bag.
    """
    log = pd.read_csv(csvPath)
    try:
        with rosbag.Bag(outputBag, 'w') as bag:
            for row in range(log.shape[0]):

                # Populate the data elements for time in seconds
                timestamp = rospy.Time.from_sec(log['Time(seconds)'][row])
                altitude_msg = Altitude()
                altitude_msg.header.stamp = timestamp
            
                # Populate the data elements for altitude in meters
                # e.g. altitude_msg.relative = df['Relative'][row]
                altitude_msg.local = log['Altitude(meters)'][row]

                bag.write(TOPIC, altitude_msg, timestamp)
                print(bag)

    except: 
        print("Error: Unable to open CSV file!")
        sys.exit()

    finally:
        bag.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog = 'csv2rosbag.py', description='Creates a ROS bag file from a CSV file.')
    parser.add_argument('-csvPath', type=str, required=False, help='Path or name of the CSV file', default = None)
    parser.add_argument('-outputBag', type=str, required = False, help='Name of the ROS bag file converted from the CSV file', default = None)
    args = parser.parse_args()
    csv2rosbag(args.csvPath, args.outputBag)

