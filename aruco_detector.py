# -*- coding: utf-8 -*-
"""
Created on Tue Sep 1 12:22:46 2019

@author: R. Erdem Uysal
company: Upteko
"""


import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco

# Import other Python modules
from calibration_store import load_coefficients
from average_center_points import calculate_avg
from draw import draw


cap = cv2.VideoCapture("50_meters.mp4")
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
  
def aruco_detector(matrix_coefficients, distortion_coefficients):
    while True:
        ret, frame = cap.read()
        #frame = cv2.imread('ship.png')
        #frame = cv2.resize(frame, (1080, 720)) # For a spesific resize operation
        #frame = cv2.resize(frame, (720, 480)) # For a spesific resize operation

        # Operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Specify marker size as 4x4, 5x5, 6x6
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
        # Lists of ids and the corners beloning to each marker
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=matrix_coefficients,
                                                                distCoeff=distortion_coefficients)
        

        height, width, channels = frame.shape
        print("Height: " + str(height) + " ___ " + "Width: " + str(width))
        print("Length of detected markers ", ids)
        
        px1 = 70 # Pixel height distance from the corners of ArUco
        px2 = 110 # Pixel height distance from the corners of ArUco
        """
        # These variables will be set for marking on video according to ship specifications.
        px1_h = 70 # Pixel height distance from the corners of ArUco
        px1_w = 210 # Pixel width distance from the center of ArUco
        px2_h = 110 # Pixel height distance from the center of ArUco
        px2_w = 330 # Pixel width distance from the center of ArUco
        """
        
        try:
            c_x, c_y, x_coo, y_coo = get_coordinates(corners, ids, frame)       
            gsd, distance = get_gsd(px = 45, height = 50, fov = 2.8, pix_w = 1280)
    
            # Average center points of 3 ArUco markers
            av_cx, av_cy = calculate_avg(ids, c_x, c_y)
            draw(frame, px1, px2, av_cx, av_cy, c_x, c_y, x_coo, y_coo)
    
            if np.all(ids is not None):  # If there are markers found by detector
                for i in range(0, len(ids)):  # Iterate in markers
                    # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                               distortion_coefficients)
                    (rvec - tvec).any()  # get rid of that nasty numpy value array error
                    aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                    aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw axis
                    
                    cv2.putText(frame, "id"+str(ids[i]), (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (25,0,200), 1)

        except:
            if ids is None or len(ids) == 0:
                print("******************************************************")
                print("*************** Marker Detection Failed **************")
                print("******************************************************")
            
        # Display the resulting frame
        cv2.imshow('frame', frame)
        # Wait 3 mili seconds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Press 'q' for quit
            break
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
def get_coordinates(corners, ids, frame):
    """
    Getting coordinates of each aruco marker's corner
    
    corners[0][0][0][0]
    index 1. represent the identity of marker
    index 2. represent the identiy of corner starts from 0 (Corner 0, Corner 1, Corner 2, Corner 3)
    index 3. same with index 2 but more suitable for processing
    index 4. represent the axis (X or Y)
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

        
def get_gsd(px, height, fov = 2.8, pix_w = 1280):
    """ 
    Calculation of ground sampling distance
    """
    gsd = (2 * height * math.tan(fov / 2)) / pix_w
    gsd = 0.06        
    
    print("\nGround Sampling Distance: %.2f m/px" %gsd)
    print("------------------------------------------------------")
    distance = (px * gsd)
    print("Distance from marker to Side 1: %.2f" %distance)
    print("------------------------------------------------------\n")
        
    return gsd, distance


if __name__ == '__main__':
    matrix_coefficients, distortion_coefficients = load_coefficients('left_cam.yml')
    aruco_detector(matrix_coefficients, distortion_coefficients)
    
    
    
    
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
