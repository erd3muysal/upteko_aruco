import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco
from calibration_store import load_coefficients

cap = cv2.VideoCapture(0)

def aruco_detector(matrix_coefficients, distortion_coefficients):
    while True:
        ret, frame = cap.read()
        frame = cv2.imread('ship.png')
        frame = cv2.resize(frame, (1080, 720)) # For a spesific resize operation

        # Operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
        # Lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=matrix_coefficients,
                                                                distCoeff=distortion_coefficients)
        
        """
        ### Simple way to find coordinates of each corner ###
        
        print(corners[0][0])
        print('x1:', corners[0][0][0][0])
        print('x2:', corners[0][0][1][0])
        print('x3:', corners[0][0][2][0]) 
        print('x4:', corners[0][0][3][0]) 
        print('y1:', corners[0][0][0][1]) 
        print('y2:', corners[0][0][1][1]) 
        print('y3:', corners[0][0][2][1]) 
        print('y4:', corners[0][0][3][1])
        
        for each in corners[0][0]:
            print(each[1])
    
        print("\nids:", ids)
        print("\ntotal number of ids:", len(ids))
        print("\ncorners:", corners)
        print("\nBirincinin kosesi: ---ID:13---", corners[-1])        
        print("\nIkincinin kosesi: ---ID:3---", corners[-2])
        print("\nUcuncunun kosesi: ---ID:23---", corners[-3])
        """
        print(corners)
        
        x_coo, y_coo = get_coordinates(corners, ids, frame)                              
        px = 70
        gsd = get_gsd(height = 125, fov = 0.87, pix_w = 895)
        #gsd = 0.13397129186
        
        print("\nGround Sampling Distance: ", gsd)
        print("------------------------------------------------------")
        distance = int(px*gsd)
        print("Distance from marker to Side 1: ", distance)
        print("------------------------------------------------------\n")
        
        cv2.line(frame,(int(x_coo["x0"]), int(y_coo["y0"])-px),(int(x_coo["x1"]), int(y_coo["y1"])-px),(25,25,200),5)
        cv2.line(frame,(int(x_coo["x0"])-px, int(y_coo["y0"])),(int(x_coo["x3"])-px, int(y_coo["y3"])),(25,25,200),5)
        cv2.line(frame,(int(x_coo["x1"])+px, int(y_coo["y1"])),(int(x_coo["x2"])+px, int(y_coo["y2"])),(25,25,200),5)
        cv2.line(frame,(int(x_coo["x2"]), int(y_coo["y2"])+px),(int(x_coo["x3"]), int(y_coo["y3"])+px),(25,25,200),5)
        
        cv2.putText(frame, 'Side 1 is', (int((x_coo["x0"]+x_coo["x1"])/2), int((y_coo["y0"]+y_coo["y1"])/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (25,200,200), 2)
        cv2.putText(frame, 'Back', (int((x_coo["x0"]+x_coo["x3"])/2), int((y_coo["y0"]+y_coo["y3"])/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (25,200,200), 2)
        cv2.putText(frame, 'Front', (int((x_coo["x1"]+x_coo["x2"])/2), int((y_coo["y1"]+y_coo["y3"])/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (25,200,200), 2)
        cv2.putText(frame, 'Side 2', (int((x_coo["x2"]+x_coo["x3"])/2), int((y_coo["y2"]+y_coo["y3"])/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (25,200,200), 2)
        

        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                           distortion_coefficients)
                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
                
                c_x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4 # x coordinate of marker's center
                c_y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4 # y coordinate of marker's center
                cv2.putText(frame, "id"+str(ids[i]), (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (25,0,200), 1)

        # Display the resulting frame
        cv2.imshow('frame', frame)
        # Wait 3 mili seconds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
"""  old version
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

def get_coordinates(corners, ids, frame):
    #Getting coordinates of each corner of aruco marker
    x_coo = {}
    y_coo = {}
    index = 0

    for i in range(0, len(ids)):
        for coo in corners[i][0]:
            x_coo["x{0}".format(index)] = coo[0]
            print(x_coo)
            y_coo["y{0}".format(index)] = coo[1]
            print(y_coo)
            index = index + 1
            
            c_x = (corners[i][0][0][0] + corners[i][0][1][0] + corners[i][0][2][0] + corners[i][0][3][0]) / 4 # x coordinate of marker's center
            c_y = (corners[i][0][0][1] + corners[i][0][1][1] + corners[i][0][2][1] + corners[i][0][3][1]) / 4 # y coordinate of marker's center
            print("X Coo",i,":", c_x)
            print("Y Coo:", c_y)
            cv2.putText(frame, "X", (int(c_x), int(c_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (25,0,200), 1)
            
        return x_coo, y_coo
        
"""
        print(corners[0][0])
        print('x1:', corners[0][0][0][0])
        print('x2:', corners[0][0][1][0])
        print('x3:', corners[0][0][2][0]) 
        print('x4:', corners[0][0][3][0]) 
        print('y1:', corners[0][0][0][1]) 
        print('y2:', corners[0][0][1][1]) 
        print('y3:', corners[0][0][2][1]) 
        print('y4:', corners[0][0][3][1])
"""
def get_gsd(height, fov, pix_w = 895):
    """ Calculation of ground sampling distance"""
    gsd = (2 * height * math.tan(fov / 2)) / pix_w
    return gsd


if __name__ == '__main__':
    matrix_coefficients, distortion_coefficients = load_coefficients('left_cam.yml')
    aruco_detector(matrix_coefficients, distortion_coefficients)
