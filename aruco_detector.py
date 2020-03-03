# -*- coding: utf-8 -*-
"""
Created on Tue Sep 3 12:22:46 2019

@author: R. Erdem Uysal
company: Upteko
"""

import numpy as np
import math
import sys
import argparse
import cv2
import cv2.aruco as aruco
import rospy
from camera_calibration import Camera
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from mavros_msgs.msg import Altitude

class Marker(object):

    def __init__(self, markerSize = 0.8, vesselLength = 25, vesselWidth = 50):
        self.cam = Camera()
        self.matrix_coefficients, self.distortion_coefficients = self.cam.loadCoefficients("cam.yml")
        self.markerSize = markerSize
        self.vesselLength = vesselLength
        self.vesselWidth = vesselWidth
        self.altitude = 0
        self.alt_sub = rospy.Subscriber("/mavros/altitude", Altitude, self.altitude_callback)

    def altitude_callback(self, data):
        self.altitude = data.local
        print("Local Altitude: ", self.altitude)

    def arucoDetector(self):
        try:
             cap = cv2.VideoCapture("data/FlightRecord50to30.MP4")
        except (cap.isOpened() == False):
            print("Error opening video stream or file")
            sys.exit()

        while True:
            retval, frame = cap.read()
            #frame = cv2.imread('ship.png')
            frame = cv2.resize(frame, (720, 480)) # (720, 480) or for a spesific resize operation

            # Operations on the frame come here
            if retval is True:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
            else:
                break

            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Specify marker size as 4x4, 5x5 or 6x6
            parameters = aruco.DetectorParameters_create()  # Marker detection parameters
            # Lists of ids and the corners beloning to each marker
            corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                    parameters = parameters,
                                                                    cameraMatrix = self.matrix_coefficients,
                                                                    distCoeff = self.distortion_coefficients)

            #print(self.matrix_coefficients, self.distortion_coefficients )

            height, width, channels = frame.shape
            print("Height x Width -> " + str(height) + "x" + str(width))

            try:
                if np.all(ids is not None):  # If there are markers found by detector
                    for i in range(0, len(ids)):  # Iterate in markers
                        if ids[i] == 1:
                            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.matrix_coefficients,
                                        self.distortion_coefficients)
                            (rvec - tvec).any()  # Get rid of that nasty numpy value array error
                            aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                            aruco.drawAxis(frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)  # Draw axis

                            c_x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4 # X coordinate of marker's center
                            c_y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4 # Y coordinate of marker's center
                            #cv2.putText(frame, "id"+str(ids[i]), (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,225,250), 2)

                            # Get center and coordinates
                            c_x, c_y, x_coo, y_coo = self.getCoordinates(corners, ids, frame)
                            # Average center points of 3 ArUco markers
                            av_cx, av_cy = self.calculateAvg(ids, c_x, c_y)

                            # Calculate ground sampling distance
                            gsd = self.getGSD(self.altitude, pix_w = 1080, fov = 2.8)
                            # Calculate ground sampling distance from marker's lengths
                            #alt_gsd = self.altGSD(x_coo, y_coo)
                            # Call draw module
                            frame = self.drawLine(frame, ids, gsd, av_cx, av_cy, c_x, c_y, x_coo, y_coo, self.markerSize, self.vesselLength, self.vesselWidth, d4 = 5)

                        else:
                            print("Unapropriate marker")
                            break

            except:
                if ids is None or len(ids) == 0:
                    print("******************************************************")
                    print("*************** Marker Detection Failed **************")
                    print("******************************************************")
            try:
                pass
            except np.all(ids is not None):
                pass

            # Find OpenCV version
            (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

            if int(major_ver)  < 3 :
                fps = cap.get(cv2.cv.CV_CAP_PROP_FPS)
                fps = str(round(fps,2))
                print("Frames Per Second: {0}".format(fps))
                cv2.putText(frame, "FPS: " + fps, (1, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,225,250), 1,5)

            else :
                fps = (cap.get(cv2.CAP_PROP_FPS))
                fps = str(round(fps,2))
                print("Frames Per Second: {0}".format(fps))
                cv2.putText(frame, "FPS: " + fps, (1, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50,225,250), 1,5)

            # Display the resulting frame
            cv2.imshow('frame', frame)

            # Wait 3 mili seconds for an interaction. Check the key and do the corresponding job.
            key = cv2.waitKey(3) & 0xFF
            if key == ord('q'):  # Press 'q' for quit
                break

        # When everything is done, release the capture
        cap.release()
        cv2.destroyAllWindows()

    def getCoordinates(self, corners, ids, frame):
        """
        Getting coordinates of each aruco marker's corner

        Format: corners[0][0][0][0]
        1st index represent the identity of marker
        2nd index represent the identiy of corner starts from 0 (Corner 0, Corner 1, Corner 2, Corner 3)
        3rd index same with index 2 but more suitable for processing
        4th index represent the axis (X or Y)
        """
        x_coo = {} # Dictionary that stores X coordinates of each aruco marker's corner
        y_coo = {} # Dictionary that stores Y coordinates of each aruco marker's corner
        c_x = {}   # Dictionary that stores X coordinates of each aruco marker's center
        c_y = {}   # Dictionary that stores Y coordinates of each aruco marker's center
        index = 0

        for i in range(0, len(ids)): # Iterate as number of ArUco markers
            c_x["c_x{0}".format(i)] = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4 # Formula for X coordinates of each aruco marker's center
            c_y["c_y{0}".format(i)] = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4 # Formula for Y coordinates of each aruco marker's center
            for coo in corners[i][0]: # Iterate through all corners start from corner[0]
                x_coo["x{0}".format(index)] = coo[0] # coo[0] represents the X axis
                #print(x_coo)
                y_coo["y{0}".format(index)] = coo[1] # coo[1] represents the Y axis
                #print(y_coo)
                index = index + 1

        return c_x, c_y, x_coo, y_coo

    def calculateAvg(self, ids, c_x, c_y):
        """
        This module is basically finds the average centroid of multiple ArUco markers from its corner coordinates.
        """
        if(len(ids) == 1):
            pass
            av_cx = c_x['c_x0']
            av_cy = c_y['c_y0']

        elif(len(ids) == 2):
            av_cx = (c_x['c_x0'] + c_x['c_x1']) / 2
            av_cy = (c_y['c_y0'] + c_y['c_y1']) / 2

        elif(len(ids) == 3):
            av_cx = (c_x['c_x0'] + c_x['c_x1'] + c_x['c_x2']) / 3
            av_cy = (c_y['c_y0'] + c_y['c_y1'] + c_y['c_y2']) / 3

        elif(len(ids) == 4):
            av_cx = (c_x['c_x0'] + c_x['c_x1'] + c_x['c_x2'] + c_x['c_x3']) / 4
            av_cy = (c_y['c_y0'] + c_y['c_y1'] + c_y['c_y2'] + c_y['c_y3']) / 4

        else:
            print("Too many ArUco markers found!")

        return av_cx, av_cy

    def getGSD(self, altitude, pix_w, fov = 2.8):
        """
        Calculation of ground sampling distance
        """
        gsd = (2 * altitude * math.tan(fov / 2)) / pix_w
        gsd = round(gsd, 2)
        print("\nGround Sampling Distance: %.2f m/px" %gsd)
        print("------------------------------------------------------")

        return gsd

    def altGSD(self, x_coo, y_coo):
        """
        (Alternative way to find GSD)
        Calculation of ground sampling distance from marker size
        """
        L1 = abs(int(x_coo["x0"]-x_coo["x1"]))
        L2 = abs(int(y_coo["y1"]-y_coo["y2"]))
        L3 = abs(int(x_coo["x2"]-x_coo["x3"]))
        L4 = abs(int(y_coo["y3"]-y_coo["y0"]))
        avg_L = (L1+L2+L3+L4) / 4

        alt_gsd = self.markerSize / avg_L

        print("\nAlternative Ground Sampling Distance: %.2f m/px" %alt_gsd)

        return alt_gsd

    def drawLine(self, frame, ids, gsd, av_cx, av_cy, c_x, c_y, x_coo, y_coo, markerSize, vesselLength, vesselWidth, d4):
        """
        This module performs processing related with marking, drawing and printing according to ArUco markers.
        """
        overlay = frame.copy()
        output = frame.copy()

        font = cv2.FONT_HERSHEY_SIMPLEX
        alpha = 0.90 # Transparency value
        beta = 10  # Contrast value

        textSize1= 1
        textSize2= 1
        thickness = 0.5

        d1 = round(((self.vesselLength - self.markerSize) / 2))
        d2 = round(((self.vesselWidth - self.markerSize) / 3))
        d3 = round((2*(self.vesselWidth - self.markerSize) / 3))
        d4 = round(d4)

        print("D1:", d1)
        print("D2:", d2)
        print("D3:", d3)
        print("D4:", d4)

        px1 = int(d1 /gsd )
        px2 = int(d2 /gsd )
        px3 = int(d3 /gsd )
        px4 = int(d4 /gsd )

        # Drawing a cross to the center point of the average of all ArUco markers.
        # cv2.putText(frame, "X", (int(av_cx), int(av_cy)), font, 0.75, (25,0,200), 1)

        # Printing labels of the sides belongs to ship
        """
        cv2.putText(overlay, 'Side 1', (int(c_x["c_x0"]), int(c_y["c_y0"])-(px1)), font, 0.75, (0,255,0), textSize1

        cv2.putText(overlay, 'Back', (int(c_x["c_x0"])-(px1), int(c_y["c_y0"])), font, 0.75, (0,255,0), textSize1

        cv2.putText(overlay, 'Front', (int(c_x["c_x0"])+(px1), int(c_y["c_y0"])), font, 0.75, (0,255,0), textSize1

        cv2.putText(overlay, 'Side 2', (int(c_x["c_x0"]), int(c_y["c_y0"])+(px1)), font, 0.75, (0,255,0), textSize1

        """
        #1 Drawing first lines on marker --> 1st one: 10 meter
        cv2.line(overlay, (int(x_coo["x0"]), int(y_coo["y0"])-px1), (int(x_coo["x1"]), int(y_coo["y1"])-px1), (25,25,200),textSize1)

        cv2.putText(overlay, ("D1:"+str(d1)+"m"), (int(x_coo["x0"]), int(y_coo["y0"])-px1), font, thickness, (50,225,250), 1)

        cv2.line(overlay, (int(x_coo["x0"])+px3, int(y_coo["y0"])), (int(x_coo["x3"])+px3, int(y_coo["y3"])), (25,25,200),textSize1)

        cv2.putText(overlay, ("D3:"+str(d3)+"m"), (int(x_coo["x0"])+px3, int(y_coo["y0"])), font, thickness, (50,225,250), textSize2)


        cv2.line(overlay, (int(x_coo["x1"])-px2, int(y_coo["y1"])), (int(x_coo["x2"])-px2, int(y_coo["y2"])), (25,25,200),textSize1)

        cv2.putText(overlay, ("D2:"+str(d2)+"m"), (int(x_coo["x1"])-px2, int(y_coo["y1"])), font, thickness, (50,225,250), textSize2)


        cv2.line(overlay, (int(x_coo["x2"]), int(y_coo["y2"])+px1), (int(x_coo["x3"]), int(y_coo["y3"])+px1), (25,25,200),textSize1)

        cv2.putText(overlay, ("D1:"+str(d1)+"m"), (int(x_coo["x2"]), int(y_coo["y2"])+px1), font, thickness, (50,225,250), textSize2)



        #2 Drawing second lines on marker--> 2nd one: 20 meter
        cv2.line(overlay, (int(x_coo["x0"]), int(y_coo["y0"])-(px1+px4)),(int(x_coo["x1"]), int(y_coo["y1"])-(px1+px4)),(25,25,200),textSize1)

        cv2.putText(overlay, ("D4:"+str(d4)+"m"), (int(x_coo["x0"]), int(y_coo["y0"])-(px1+px4)), font, thickness, (0,255,0), textSize2)


        cv2.line(overlay, (int(x_coo["x0"])+(px3+px4), int(y_coo["y0"])),(int(x_coo["x3"])+(px3+px4), int(y_coo["y3"])),(25,25,200),textSize1)

        cv2.putText(overlay, ("D4:"+str(d4)+"m"), (int(x_coo["x0"])+(px3+px4), int(y_coo["y0"])), font, thickness, (0,255,0), textSize2)


        cv2.line(overlay, (int(x_coo["x1"])-(px2+px4), int(y_coo["y1"])),(int(x_coo["x2"])-(px2+px4), int(y_coo["y2"])),(25,25,200),textSize1)

        cv2.putText(overlay, ("D4:"+str(d4)+"m"), (int(x_coo["x1"])-(px2+px4), int(y_coo["y1"])), font, thickness, (0,255,0), textSize2)


        cv2.line(overlay, (int(x_coo["x2"]), int(y_coo["y2"])+(px1+px4)),(int(x_coo["x3"]), int(y_coo["y3"])+(px1+px4)),(25,25,200),textSize1)

        cv2.putText(overlay, ("D4:"+str(d4)+"m"), (int(x_coo["x2"]), int(y_coo["y2"])+(px1+px4)), font, thickness, (0,255,0), textSize2)


        #3 Drawing third lines on marker--> 2nd one: 40 meter
        cv2.line(overlay, (int(x_coo["x0"]), int(y_coo["y0"])-(2*px4+px1)),(int(x_coo["x1"]), int(y_coo["y1"])-(2*px4+px1)),(25,25,200),textSize1)

        cv2.putText(overlay, ("D4:"+str(d4)+"m"), (int(x_coo["x0"]), int(y_coo["y0"])-(2*px4+px1)), font,thickness,  (0,255,0), textSize2)


        cv2.line(overlay, (int(x_coo["x0"])+(2*px4+px3), int(y_coo["y0"])),(int(x_coo["x3"])+(2*px4+px3), int(y_coo["y3"])),(25,25,200),textSize1)

        cv2.putText(overlay, ("D4:"+str(d4)+"m"), (int(x_coo["x0"])+(2*px4+px3), int(y_coo["y0"])), font, thickness, (0,255,0), textSize2)


        cv2.line(overlay, (int(x_coo["x1"])-(2*px4+px2), int(y_coo["y1"])),(int(x_coo["x2"])-(2*px4+px2), int(y_coo["y2"])),(25,25,200),textSize1)

        cv2.putText(overlay, ("D4:"+str(d4)+"m"), (int(x_coo["x1"])-(2*px4+px2), int(y_coo["y1"])), font, thickness, (0,255,0), textSize2)


        cv2.line(overlay, (int(x_coo["x2"]), int(y_coo["y2"])+(2*px4+px1)),(int(x_coo["x3"]), int(y_coo["y3"])+(2*px4+px1)),(25,25,200),textSize1)

        cv2.putText(overlay, ("D4:"+str(d4)+"m"), (int(x_coo["x2"]), int(y_coo["y2"])+(px1+px4)), font, thickness, (0,255,0), textSize2)


        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, beta)

        return frame

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Line Drawing Relative to Marker')
    parser.add_argument('--markerSize', type=float, required=False, help="Marker's size in cm")
    parser.add_argument('--vesselLength', type=int, required=False, help="Vessel's length in m")
    parser.add_argument('--vesselWidth', type=int, required=False, help="Vessel's width in m")

    args = parser.parse_args()
    #marker = Marker(args.markerSize, args.vesselLength, args.vesselWidth)
    rospy.init_node('marker_node')
    rate = rospy.Rate(5)
    marker = Marker()
    marker.arucoDetector()
    rospy.sleep (1) # wait until everything is running
    # loop until shutdown
