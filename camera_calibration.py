import numpy as np
import cv2
import glob

def calibrate(path, prefix, image_format, size, width = 7, height = 6):
	# Termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

	# Prepare object points like (0, 0, 0), (1, 0, 0), (2, 0, 0) ... (6, 5, 0)
	obj_p = np.zeros((height*weight, 3), np.float)
	obj_p[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

	#Arrays to stroe object points and image point from all the images.
	obj_points = [] # 3D points in real world space
	img_points = [] # 2D points in image plane

	images = glob.glob('*.jpg')

	for image in images:
		img = cv2.imread(image)
	 	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

		# If found, add object points, image points (after refining them)
		if ret == True
			obj_points.append(obj_p)
			
			corners_ = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
			img_points.append(corners)

			# Draw and display the corners
			image = cv2.drawChessboardCorners(img, (width, height), corners_, ret)

	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
	return [ret, mtx, dist, rvecs, tvecs]	
