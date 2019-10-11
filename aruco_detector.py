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
<<<<<<< HEAD
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
=======
# Import other Python modules
from camera_calibration import Calibration

class Marker(object):
    
    def __init__(self, matrix_coefficients, distortion_coefficients, markerSize = 0.8, vesselLength = 28, vesselWidth = 75):
        self.matrix_coefficients = matrix_coefficients
        self.distortion_coefficients = distortion_coefficients
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
        self.markerSize = markerSize
        self.vesselLength = vesselLength
        self.vesselWidth = vesselWidth
        
    def arucoDetector(self):
        try:
             cap = cv2.VideoCapture("20_meters.mp4")
        except (cap.isOpened() == False): 
            print("Error opening video stream or file")
            sys.exit()
        
        while True:
            retval, frame = cap.read()
            #frame = cv2.imread('ship.png')
<<<<<<< HEAD
            frame = cv2.resize(frame, (1080, 720)) # (720, 480) or for a spesific resize operation
=======
            #frame = cv2.resize(frame, (1080, 720)) # (720, 480) or for a spesific resize operation
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
            
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
            
            height, width, channels = frame.shape
<<<<<<< HEAD
            print("Height x Width -> " + str(height) + "x" + str(width))
=======
            print("Height x Width: " + str(height) + "x" + str(width))
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
            
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
                            gsd = self.getGSD(height = 50, fov = 2.8, pix_w = 1080)
                            # Calculate ground sampling distance from marker's lengths
<<<<<<< HEAD
                            alt_gsd = self.altGSD(x_coo, y_coo)
                            
                            # Call draw module  
                            frame = self.drawLine(frame, ids, gsd, av_cx, av_cy, c_x, c_y, x_coo, y_coo, self.markerSize, self.vesselLength, self.vesselWidth, d4 = 5)
=======
                            a_gsd = self.altGSD(x_coo, y_coo)
                            
                            # Call draw module  
                            frame = self.draw(frame, ids, gsd, av_cx, av_cy, c_x, c_y, x_coo, y_coo, self.markerSize, self.vesselLength, self.vesselWidth, d4 = 5)
>>>>>>> 17468482624b47b71e544229f55310cd9d892513

                        else:
                            print("Unapropriate marker")
                            break
     
            except:
                if ids is None or len(ids) == 0:
                    print("******************************************************")
                    print("*************** Marker Detection Failed **************")
                    print("******************************************************")
    
            # Find OpenCV version
            (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
             
            if int(major_ver)  < 3 :
                fps = cap.get(cv2.cv.CV_CAP_PROP_FPS)
                fps = str(round(fps,2))
                print("Frames Per Second: {0}".format(fps))
                cv2.putText(frame, "FPS: " + fps, (1, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,225,250), 1)
    
            else :
                fps = (cap.get(cv2.CAP_PROP_FPS))
                fps = str(round(fps,2))
                print("Frames Per Second: {0}".format(fps))
                cv2.putText(frame, "FPS: " + fps, (1, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,225,250), 2)
    
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
<<<<<<< HEAD
                #print(x_coo)
                y_coo["y{0}".format(index)] = coo[1] # coo[1] represents the Y axis
                #print(y_coo)
=======
                print(x_coo)
                y_coo["y{0}".format(index)] = coo[1] # coo[1] represents the Y axis
                print(y_coo)
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
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

    def getGSD(self, height, fov = 2.8, pix_w = 1280):
        """ 
        Calculation of ground sampling distance
        """
        gsd = (2 * height * math.tan(fov / 2)) / pix_w
<<<<<<< HEAD
        gsd = 0.06        
=======
        ###########################################################################
        gsd = 0.06        
        ###########################################################################
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
        print("\nGround Sampling Distance: %.2f m/px" %gsd)
        print("------------------------------------------------------")
    
        return gsd
    
    def altGSD(self, x_coo, y_coo):
        """
        Calculation of ground sampling distance from marker size
        """
<<<<<<< HEAD
=======
        markerSize = 0.80
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
        L1 = abs(int(x_coo["x0"]-x_coo["x1"]))
        L2 = abs(int(y_coo["y1"]-y_coo["y2"]))
        L3 = abs(int(x_coo["x2"]-x_coo["x3"]))
        L4 = abs(int(y_coo["y3"]-y_coo["y0"]))
        avg_L = (L1+L2+L3+L4) / 4
    
<<<<<<< HEAD
        alt_gsd = self.markerSize / avg_L
        
        print("\nGround Sampling Distance: %.2f m/px" %alt_gsd)
    
        return alt_gsd

    def drawLine(self, frame, ids, gsd, av_cx, av_cy, c_x, c_y, x_coo, y_coo, markerSize, vesselLength, vesselWidth, d4):
=======
        a_gsd = markerSize / avg_L
        
        print("\nGround Sampling Distance: %.2f m/px" %a_gsd)
    
        return a_gsd

    def draw(self, frame, ids, gsd, av_cx, av_cy, c_x, c_y, x_coo, y_coo, d4, vesselLength, vesselWidth, markerSize):
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
        """
        This module performs processing related with marking, drawing and printing according to ArUco markers.
        """
        overlay = frame.copy()
        output = frame.copy()
    
        font = cv2.FONT_HERSHEY_SIMPLEX
        alpha = 0.7 # Transparency value
        beta = 10  # Contrast value   
        thickness = 2
        
<<<<<<< HEAD
        d1 = (self.vesselLength - self.markerSize) /2
        d2 = (self.vesselWidth - self.markerSize) /3
        d3 = (2*(self.vesselWidth - self.markerSize)) /3
=======
        d1 = (vesselLength - markerSize) /2
        d2 = (vesselWidth - markerSize) /3
        d3 = (2*(vesselWidth - markerSize)) /3
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
    
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
        cv2.putText(overlay, 'Side 1', (int(c_x["c_x0"]), int(c_y["c_y0"])-(px1)), font, 0.75, (0,255,0), thickness)
        cv2.putText(overlay, 'Back', (int(c_x["c_x0"])-(px1), int(c_y["c_y0"])), font, 0.75, (0,255,0), thickness)
        cv2.putText(overlay, 'Front', (int(c_x["c_x0"])+(px1), int(c_y["c_y0"])), font, 0.75, (0,255,0), thickness)
        cv2.putText(overlay, 'Side 2', (int(c_x["c_x0"]), int(c_y["c_y0"])+(px1)), font, 0.75, (0,255,0), thickness)
        
        # Drawing first lines on 1st marker --> 1st one: 10 meter 
        cv2.line(overlay, (int(x_coo["x0"]), int(y_coo["y0"])-px1), (int(x_coo["x1"]), int(y_coo["y1"])-px1), (25,25,200),thickness)
        cv2.line(overlay, (int(x_coo["x0"])+px3, int(y_coo["y0"])), (int(x_coo["x3"])+px3, int(y_coo["y3"])), (25,25,200),thickness)
        cv2.line(overlay, (int(x_coo["x1"])-px2, int(y_coo["y1"])), (int(x_coo["x2"])-px2, int(y_coo["y2"])), (25,25,200),thickness)
        cv2.line(overlay, (int(x_coo["x2"]), int(y_coo["y2"])+px1), (int(x_coo["x3"]), int(y_coo["y3"])+px1), (25,25,200),thickness)
        
        # Drawing second lines on 1st marker--> 2nd one: 20 meter
<<<<<<< HEAD
        cv2.line(overlay,(int(x_coo["x0"]), int(y_coo["y0"])-(px1+px4)),(int(x_coo["x1"]), int(y_coo["y1"])-(px1+px4)),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x0"])+(px3+px4), int(y_coo["y0"])),(int(x_coo["x3"])+(px3+px4), int(y_coo["y3"])),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x1"])-(px2+px4), int(y_coo["y1"])),(int(x_coo["x2"])-(px2+px4), int(y_coo["y2"])),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x2"]), int(y_coo["y2"])+(px1+px4)),(int(x_coo["x3"]), int(y_coo["y3"])+(px1+px4)),(25,25,200),thickness)    
        
        # Drawing second lines on 1st marker--> 2nd one: 40 meter
        cv2.line(overlay,(int(x_coo["x0"]), int(y_coo["y0"])-(2*px4+px1)),(int(x_coo["x1"]), int(y_coo["y1"])-(2*px4+px1)),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x0"])+(2*px4+px3), int(y_coo["y0"])),(int(x_coo["x3"])+(2*px4+px3), int(y_coo["y3"])),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x1"])-(2*px4+px2), int(y_coo["y1"])),(int(x_coo["x2"])-(2*px4+px2), int(y_coo["y2"])),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x2"]), int(y_coo["y2"])+(2*px4+px1)),(int(x_coo["x3"]), int(y_coo["y3"])+(2*px4+px1)),(25,25,200),thickness)    
=======
        cv2.line(overlay,(int(x_coo["x0"]), int(y_coo["y0"])+(px1+px4)),(int(x_coo["x1"]), int(y_coo["y1"])+(px1+px4)),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x0"])+(px3+px4), int(y_coo["y0"])),(int(x_coo["x3"])+(px3+px4), int(y_coo["y3"])),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x1"])-(px2+px4), int(y_coo["y1"])),(int(x_coo["x2"])-(px2+px4), int(y_coo["y2"])),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x2"]), int(y_coo["y2"])-(px1+px4)),(int(x_coo["x3"]), int(y_coo["y3"])-(px1+px4)),(25,25,200),thickness)    
        
        # Drawing second lines on 1st marker--> 2nd one: 40 meter
        cv2.line(overlay,(int(x_coo["x0"]), int(y_coo["y0"])+(2*px4+px1)),(int(x_coo["x1"]), int(y_coo["y1"])+(2*px4+px1)),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x0"])-(2*px4+px3), int(y_coo["y0"])),(int(x_coo["x3"])-(2*px4+px3), int(y_coo["y3"])),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x1"])+(2*px4+px2), int(y_coo["y1"])),(int(x_coo["x2"])+(2*px4+px2), int(y_coo["y2"])),(25,25,200),thickness)
        cv2.line(overlay,(int(x_coo["x2"]), int(y_coo["y2"])-(2*px4+px1)),(int(x_coo["x3"]), int(y_coo["y3"])-(2*px4+px1)),(25,25,200),thickness)    
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
    
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, beta)
    
        return frame
<<<<<<< HEAD
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera Calibration')
    parser.add_argument('--markerSize', type=float, required=False, help="Marker's size in cm")
    parser.add_argument('--vesselLength', type=int, required=False, help="Vessel's length in m")
    parser.add_argument('--vesselWidth', type=int, required=False, help="Vessel's width in m")

    args = parser.parse_args()
    # Do not remove here!
    #marker = Marker(args.markerSize, args.vesselLength, args.vesselWidth)
    marker = Marker()
    marker.arucoDetector()
=======
    #    # Drawing lines according to average center points --> 1st one: 5 meter
    #    cv2.line(frame,(int(av_cx)-100, int(av_cy)-(px1)),(int(av_cx)+100, int(av_cy)-(px1)),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)-(px1), int(av_cy)-100),(int(av_cx)-(px1), int(av_cy)+100),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)+(px1), int(av_cy)-100),(int(av_cx)+(px1), int(av_cy)+100),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)-100, int(y_coo["y2"])+(px1)),(int(av_cx)+100, int(av_cy)+(px1)),(25,25,200),3)
    #
    #    # Drawing lines according to average center points --> 2nd one: 10 meter
    #    cv2.line(frame,(int(av_cx)-100, int(av_cy)-(2*px1)),(int(av_cx)+100, int(av_cy)-(2*px1)),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)-(2*px1), int(av_cy)-100),(int(av_cx)-(2*px1), int(av_cy)+100),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)+(2*px1), int(av_cy)-100),(int(av_cx)+(2*px1), int(av_cy)+100),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)-100, int(y_coo["y2"])+(2*px1)),(int(av_cx)+100, int(av_cy)+(2*px1)),(25,25,200),3)
    #
    #    # Drawing lines according to average center points --> 3rd one: 15 meter
    #    cv2.line(frame,(int(av_cx)-100, int(av_cy)-(3*px1)),(int(av_cx)+100, int(av_cy)-(3*px1)),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)-(3*px1), int(av_cy)-100),(int(av_cx)-(3*px1), int(av_cy)+100),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)+(3*px1), int(av_cy)-100),(int(av_cx)+(3*px1), int(av_cy)+100),(25,25,200),3)
    #    cv2.line(frame,(int(av_cx)-100, int(y_coo["y2"])+(3*px1)),(int(av_cx)+100, int(av_cy)+(3*px1)),(25,25,200),3)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera Calibration')
    parser.add_argument('--filePath', type=str, required=True, help="Image directory path")
    parser.add_argument('--imgName', type=str, required=True, help="Image name")
    parser.add_argument('--imgFormat', type=str, required=True,  help="Image format, png/jpg")
    parser.add_argument('--squareSize', type=float, required=False, help="Chessboard square size")
    parser.add_argument('--width', type=int, required=False, help="Chessboard width size, default is 9")
    parser.add_argument('--height', type=int, required=False, help="Chessboard height size, default is 6")
    parser.add_argument('--saveFile', type=str, required=True, help="YML file to save calibration matrices")

    args = parser.parse_args()
    cam = Calibration(args.filePath, args.imgName, args.imgFormat, args.squareSize, args.width, args.height)
    matrix_coefficients, distortion_coefficients = cam.load_coefficients('cam.yml')
    marker = Marker(matrix_coefficients, distortion_coefficients)
    marker.arucoDetector()
    
    
    
    
    
###############################################################################
    ### Accessing each corner 
    #print(corners[0][0])
    #print('x1:', corners[0][0][0][0])
    #print('x2:', corners[0][0][1][0])
    #print('x3:', corners[0][0][2][0]) 
    #print('x4:', corners[0][0][3][0]) 
    #print('y1:', corners[0][0][0][1]) 
    #print('y2:', corners[0][0][1][1]) 
    #print('y3:', corners[0][0][2][1]) 
    #print('y4:', corners[0][0][3][1])
    
    
"""  
# Version 1

def get_coordinates(corners, ids):
    #Getting coordinates of each corner of aruco marker
    x_coo = {}
    y_coo = {} 
    for corner in corners:
        #print("corner", corner)
        for coo in corner:
            #print("coo", coo[0][0])
            #print("coo", coo[0][1])
            for i in range(0, 4):
                x_coo["x{0}".format(i)] = coo[i][0]
                print(x_coo)
            for j in range(0, 4):
                y_coo["y{0}".format(j)] = coo[j][1]
                print(y_coo)
    return x_coo, y_coo
"""


### Test
"""
for each in corners[0][0]:
    print(each[1])
    
print("\nids:", ids)
print("\ntotal number of ids:", len(ids))
print("\ncorners:", corners)
print("\nBirincinin kosesi: ---ID:13---", corners[-1])        
print("\nIkincinin kosesi: ---ID:3---", corners[-2])
print("\nUcuncunun kosesi: ---ID:23---", corners[-3])
"""
>>>>>>> 17468482624b47b71e544229f55310cd9d892513
