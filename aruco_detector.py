import numpy as np
import cv2
import cv2.aruco as aruco

cap = cv2.VideoCapture()

def track(instrinsic, extrinsic):
	while True:
		ret, frame = cap.read()
		# Operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		# Use 5x5 dictionary to find ArUco marker	
		aruco_dictionary = aruco.Dictiponary_get(aruco.DICT_5X5_250) 

		# Marker detection parameters
		parameters = aruco.DetectorParameters_create() 

		# List pf ID's and the corners beloning to each ID
		corners, ids, rejected = aruco.detectMarkers(image = gray, 
								dictionary = aruco_dictionary, 
								parameters = parameters, 
								cameraMatrix = instrinsic, 
								distCoeff = extrinsic)

		if np.all(ids is not None):
			for i in range(0, len(ids)):
				# Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
				rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, 
											   instrinsic, extrinsic)
				# Get rid of that nasty numpy value error
				(rvec - tvec).any() 
		# Draw a square around the ArUco markers
		aruco.drawDectedMarkers(frame, corners)
		# Draw axis
		aruco.drawAxis(frame, instrinsic, extrinsic, rvec, tvec, 0.01)

		# Display the resulting frame
		cv2.imshow('frame', frame)
		# Wait 3 miliseconds for an interaction. Check the key and do the corresponding job.
		key = cv2.waitKey(3)
		if key == ord('q'):
			break

	# When everything donei release the capture
	cap.release()
	cv2.destroyAllWindows()
