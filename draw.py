# -*- coding: utf-8 -*-
"""
Created on Mon Sep 15 09:28:30 2019

@author: R. Erdem Uysal
company: Upteko
"""

import cv2
"""

"""
def draw(frame, ids, gsd, av_cx, av_cy, c_x, c_y, x_coo, y_coo, d4, vesselLength, vesselWidth, markerSize):
    """
    This module performs processing related with marking, drawing and printing according to ArUco markers.
    """
    overlay = frame.copy()
    output = frame.copy()

    font = cv2.FONT_HERSHEY_SIMPLEX
    alpha = 0.7 # Transparency value
    beta = 10  # Contrast value   
    
    d1 = (vesselLength - markerSize) /2
    d2 = (vesselWidth - markerSize) /3
    d3 = (2*(vesselWidth - markerSize)) /3

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
    cv2.putText(overlay, 'Side 1', (int(c_x["c_x0"]), int(c_y["c_y0"])-(px1)), font, 0.75, (0,255,0), 2)
    cv2.putText(overlay, 'Back', (int(c_x["c_x0"])-(px1), int(c_y["c_y0"])), font, 0.75, (0,255,0), 2)
    cv2.putText(overlay, 'Front', (int(c_x["c_x0"])+(px1), int(c_y["c_y0"])), font, 0.75, (0,255,0), 2)
    cv2.putText(overlay, 'Side 2', (int(c_x["c_x0"]), int(c_y["c_y0"])+(px1)), font, 0.75, (0,255,0), 2)
    
    # Drawing first lines on 1st marker --> 1st one: 10 meter 
    cv2.line(overlay, (int(x_coo["x0"]), int(y_coo["y0"])-px1), (int(x_coo["x1"]), int(y_coo["y1"])-px1), (25,25,200),3)
    cv2.line(overlay, (int(x_coo["x0"])+px3, int(y_coo["y0"])), (int(x_coo["x3"])+px3, int(y_coo["y3"])), (25,25,200),3)
    cv2.line(overlay, (int(x_coo["x1"])-px2, int(y_coo["y1"])), (int(x_coo["x2"])-px2, int(y_coo["y2"])), (25,25,200),3)
    cv2.line(overlay, (int(x_coo["x2"]), int(y_coo["y2"])+px1), (int(x_coo["x3"]), int(y_coo["y3"])+px1), (25,25,200),3)
    
    # Drawing second lines on 1st marker--> 2nd one: 20 meter
    cv2.line(overlay,(int(x_coo["x0"]), int(y_coo["y0"])+(px1+px4)),(int(x_coo["x1"]), int(y_coo["y1"])+(px1+px4)),(25,25,200),3)
    cv2.line(overlay,(int(x_coo["x0"])+(px3+px4), int(y_coo["y0"])),(int(x_coo["x3"])+(px3+px4), int(y_coo["y3"])),(25,25,200),3)
    cv2.line(overlay,(int(x_coo["x1"])-(px2+px4), int(y_coo["y1"])),(int(x_coo["x2"])-(px2+px4), int(y_coo["y2"])),(25,25,200),3)
    cv2.line(overlay,(int(x_coo["x2"]), int(y_coo["y2"])-(px1+px4)),(int(x_coo["x3"]), int(y_coo["y3"])-(px1+px4)),(25,25,200),3)    
    
    # Drawing second lines on 1st marker--> 2nd one: 40 meter
    cv2.line(overlay,(int(x_coo["x0"]), int(y_coo["y0"])+(2*px4+px1)),(int(x_coo["x1"]), int(y_coo["y1"])+(2*px4+px1)),(25,25,200),3)
    cv2.line(overlay,(int(x_coo["x0"])-(2*px4+px3), int(y_coo["y0"])),(int(x_coo["x3"])-(2*px4+px3), int(y_coo["y3"])),(25,25,200),3)
    cv2.line(overlay,(int(x_coo["x1"])+(2*px4+px2), int(y_coo["y1"])),(int(x_coo["x2"])+(2*px4+px2), int(y_coo["y2"])),(25,25,200),3)
    cv2.line(overlay,(int(x_coo["x2"]), int(y_coo["y2"])-(2*px4+px1)),(int(x_coo["x3"]), int(y_coo["y3"])-(2*px4+px1)),(25,25,200),3)    

    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, beta)

    return frame
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
