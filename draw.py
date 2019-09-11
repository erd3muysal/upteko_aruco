# -*- coding: utf-8 -*-
"""
Created on Wed Sep 11 09:28:30 2019

@author: ASUS
"""


import cv2


def draw(frame, px1, px2, av_cx, av_cy, c_x, c_y, x_coo, y_coo):
    """
    This module performs processing related with marking, drawing and printing according to ArUco markers.
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    
    # Drawing a cross to the center point of the average of all ArUco markers.
    cv2.putText(frame, "X", (int(av_cx), int(av_cy)), font, 0.75, (25,0,200), 1)
    
    # Printing ids on the centroid of each ArUco markers.
    cv2.putText(frame, 'Side 1', (int(c_x["c_x0"]), int(c_y["c_y0"])-(px2)), font, 0.75, (25,200,200), 2)
    cv2.putText(frame, 'Back', (int(c_x["c_x0"])-(2*px2), int(c_y["c_y0"])), font, 0.75, (25,200,200), 2)
    cv2.putText(frame, 'Front', (int(c_x["c_x0"])+(2*px2), int(c_y["c_y0"])), font, 0.75, (25,200,200), 2)
    cv2.putText(frame, 'Side 2', (int(c_x["c_x0"]), int(c_y["c_y0"])+(px2)), font, 0.75, (25,200,200), 2)
    
    # Drawing lines --> 1st one: 10 meter 
    cv2.line(frame, (int(x_coo["x0"]), int(y_coo["y0"])-px1), (int(x_coo["x1"]), int(y_coo["y1"])-px1), (25,25,200),5)
    cv2.line(frame, (int(x_coo["x0"])-px1, int(y_coo["y0"])), (int(x_coo["x3"])-px1, int(y_coo["y3"])), (25,25,200),5)
    cv2.line(frame, (int(x_coo["x1"])+px1, int(y_coo["y1"])), (int(x_coo["x2"])+px1, int(y_coo["y2"])), (25,25,200),5)
    cv2.line(frame, (int(x_coo["x2"]), int(y_coo["y2"])+px1), (int(x_coo["x3"]), int(y_coo["y3"])+px1), (25,25,200),5)
    
    # Drawing lines --> 1st one: 20 meter
    cv2.line(frame,(int(x_coo["x0"]), int(y_coo["y0"])-(2*px1)),(int(x_coo["x1"]), int(y_coo["y1"])-(2*px1)),(25,25,200),5)
    cv2.line(frame,(int(x_coo["x0"])-(2*px1), int(y_coo["y0"])),(int(x_coo["x3"])-(2*px1), int(y_coo["y3"])),(25,25,200),5)
    cv2.line(frame,(int(x_coo["x1"])+(2*px1), int(y_coo["y1"])),(int(x_coo["x2"])+(2*px1), int(y_coo["y2"])),(25,25,200),5)
    cv2.line(frame,(int(x_coo["x2"]), int(y_coo["y2"])+(2*px1)),(int(x_coo["x3"]), int(y_coo["y3"])+(2*px1)),(25,25,200),5)
    

