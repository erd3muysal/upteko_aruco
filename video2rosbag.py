# -*- coding: utf-8 -*-
# @author: R. Erdem Uysal

import time, sys, os
import roslib, rospy
import cv2
import argparse
from ros import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

TOPIC = 'camera/image_raw'
roslib.load_manifest('sensor_msgs')

def video2rosbag(videoPath, outputBag):
    """
    Creates a ROS bag file from a video file
    """
    
    try:
        bag = rosbag.Bag(outputBag, 'w')
        cap = cv2.VideoCapture(videoPath)
        bridge = CvBridge()
        
        prop_fps = (cap.get(cv2.CAP_PROP_FPS))
        if prop_fps != prop_fps or prop_fps <= 1e-2:
            print("Warning: Unable to get FPS. Default value for the FPS is 25.")
            prop_fps = 25
        
        #prop_fps = 24
        ret = True
        frame_id = 0
        while(ret):
            ret, frame = cap.read()
            if not ret:
                break

            timestamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
            frame_id += 1
            image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image.header.stamp = timestamp
            image.header.frame_id = "camera"
            bag.write(TOPIC, image, timestamp)
        cap.release()

    except (cap.isOpened() == False): 
        print("Error: Opening video stream or file failed!")
        sys.exit()

    finally:
        bag.close()
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog = 'video2rosbag.py', description='Creates a ROS bag file from a video file.')
    parser.add_argument('-videoPath', type=str, required=False, help='Path or name of the video file', default = None)
    parser.add_argument('-outputBag', type=str, required = False, help='Name of the ROS bag file converted from the video file', default = None)
    args = parser.parse_args()
    video2rosbag(args.videoPath, args.outputBag)