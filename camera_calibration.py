# -*- coding: utf-8 -*-
"""
Created on Wed Sep 11 09:28:30 2019

@author: R. Erdem Uysal
company: Upteko
"""

import numpy as np
import cv2
import glob
import argparse
from calibration_store import save_coefficients

class Calibration(object):
    def __init__(self, filePath, imgName, imgFormat, squareSize = 0.015, width = 9, height = 6):
        self.filePath = filePath
        self.imgName = imgName
        self.imgFormat = imgFormat
        self.squareSize = squareSize
        self.width = width
        self.height = height
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            
    def calibrate(self):
        """
        Apply camera calibration operation for images in the given directory path.

        """
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
        objp = np.zeros((self.height*self.width, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.width, 0:self.height].T.reshape(-1, 2)
        objp = objp * self.squareSize  # Create real world coords. Use your metric.
    
        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3D point in real world space
        imgpoints = []  # 2D points in image plane.
    
        # Directory path correction. Remove the last character if it is '/'
        if self.filePath[-1:] == '/':
            self.filePath = self.filePath[:-1]
    
        # Get the images
        images = glob.glob(self.filePath + '/' + self.imgName + '*.' + self.imgFormat)
        
        # Iterate through the pairs and find chessboard corners. Add them to arrays.
        # If openCV can't find the corners in an image, we discard the image.
        for fileName in images:
            img = cv2.imread(fileName)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
            # Find the chess board corners.
            retval, corners = cv2.findChessboardCorners(gray, (self.width, self.height), None)
    
            # If found, add object points, image points (after refining them)
            if retval:
                objpoints.append(objp)
    
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                imgpoints.append(corners2)
    
                # Draw and display the corners
                # Show the image to see if pattern is found ! imshow function.
                img = cv2.drawChessboardCorners(img, (self.width, self.height), corners2, retval)
    
        retval, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        
        return [retval, mtx, dist, rvecs, tvecs]

    def save_coefficients(self, path):
        """ 
        Save the camera matrix and the distortion coefficients to given path/file.
        """
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        cv_file.write("K", self.mtx)
        cv_file.write("D", self.dist)
        # Note you *release* you don't close() a FileStorage object
        cv_file.release()
        
    def load_coefficients(self, path):
        """ 
        Loads camera matrix and distortion coefficients. 
        """
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    
        # Note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        camera_matrix = cv_file.getNode("K").mat()
        dist_matrix = cv_file.getNode("D").mat()
        
        cv_file.release()
        return [camera_matrix, dist_matrix]


if __name__ == '__main__':
    # Check the help parameters to understand arguments
    parser = argparse.ArgumentParser(description='Camera Calibration')
    parser.add_argument('--filePath', type=str, required=True, help="Image directory path")
    parser.add_argument('--imgName', type=str, required=True, help="Image name")
    parser.add_argument('--imgFormat', type=str, required=True,  help="Image format, png/jpg")
    parser.add_argument('--squareSize', type=float, required=False, help="Chessboard square size")
    parser.add_argument('--width', type=int, required=False, help="Chessboard width size, default is 9")
    parser.add_argument('--height', type=int, required=False, help="Chessboard height size, default is 6")
    parser.add_argument('--saveFile', type=str, required=True, help="YML file to save calibration matrices")

    args = parser.parse_args()
    camera = Calibration(args.filePath, args.imgName, args.imgFormat, args.squareSize, args.width, args.height)
    
    # Call the calibraton and save as file. RMS is the error rate, it is better if rms is less than 0.2.
    retval, mtx, dist, rvecs, tvecs = camera.calibrate()
    save_coefficients(mtx, dist, args.saveFile)
    print("Calibration is successful.\n RMS: ", retval)
