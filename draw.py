# -*- coding: utf-8 -*-
"""
Created on Mon Sep 15 09:28:30 2019

@author: R. Erdem Uysal
company: Upteko
"""

import cv2

def draw(frame, ids, px1, px2, av_cx, av_cy, c_x, c_y, x_coo, y_coo):
    """
    This module performs processing related with marking, drawing and printing according to ArUco markers.
    """
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Drawing a cross to the center point of the average of all ArUco markers.
    cv2.putText(frame, "X", (int(av_cx), int(av_cy)), font, 0.75, (25,0,200), 1)

    # Printing labels of the sides belongs to ship    
#    cv2.putText(frame, 'Side 1', (int(c_x["c_x0"]), int(c_y["c_y0"])-(px1)), font, 0.75, (0,255,0), 2)
#    cv2.putText(frame, 'Back', (int(c_x["c_x0"])-(px1), int(c_y["c_y0"])), font, 0.75, (0,255,0), 2)
#    cv2.putText(frame, 'Front', (int(c_x["c_x0"])+(px1), int(c_y["c_y0"])), font, 0.75, (0,255,0), 2)
#    cv2.putText(frame, 'Side 2', (int(c_x["c_x0"]), int(c_y["c_y0"])+(px1)), font, 0.75, (0,255,0), 2)
    
#    # Drawing lines according to average center points --> 1st one: 20 meter
#    cv2.line(frame,(int(av_cx)-10, int(av_cy)-(2*px1)),(int(av_cx)+10, int(av_cy)-(2*px1)),(25,25,200),3)
#    cv2.line(frame,(int(av_cx)-(2*px1), int(av_cy)-10),(int(av_cx)-(2*px1), int(av_cy)+10),(25,25,200),3)
#    cv2.line(frame,(int(av_cx)+(2*px1), int(av_cy)-10),(int(av_cx)+(2*px1), int(av_cy)+10),(25,25,200),53)
#    cv2.line(frame,(int(av_cx)-10, int(y_coo["y2"])+(2*px1)),(int(av_cx)+10, int(av_cy)+(2*px1)),(25,25,200),3)
    
    if(len(ids) == 1):
        
        # Drawing first lines on 1st marker --> 1st one: 10 meter 
        cv2.line(frame, (int(x_coo["x0"]), int(y_coo["y0"])-px1), (int(x_coo["x1"]), int(y_coo["y1"])-px1), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x0"])-px1, int(y_coo["y0"])), (int(x_coo["x3"])-px1, int(y_coo["y3"])), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x1"])+px1, int(y_coo["y1"])), (int(x_coo["x2"])+px1, int(y_coo["y2"])), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x2"]), int(y_coo["y2"])+px1), (int(x_coo["x3"]), int(y_coo["y3"])+px1), (25,25,200),3)
        
        # Drawing second lines on 1st marker--> 2nd one: 20 meter
        cv2.line(frame,(int(x_coo["x0"]), int(y_coo["y0"])-(2*px1)),(int(x_coo["x1"]), int(y_coo["y1"])-(2*px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x0"])-(2*px1), int(y_coo["y0"])),(int(x_coo["x3"])-(2*px1), int(y_coo["y3"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x1"])+(2*px1), int(y_coo["y1"])),(int(x_coo["x2"])+(2*px1), int(y_coo["y2"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x2"]), int(y_coo["y2"])+(2*px1)),(int(x_coo["x3"]), int(y_coo["y3"])+(2*px1)),(25,25,200),3)
    
    if(len(ids) == 2):
        
        # Drawing first lines on 2nd marker --> 1st one: 10 meter 
        cv2.line(frame, (int(x_coo["x0"]), int(y_coo["y0"])-px1), (int(x_coo["x1"]), int(y_coo["y1"])-px1), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x0"])-px1, int(y_coo["y0"])), (int(x_coo["x3"])-px1, int(y_coo["y3"])), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x1"])+px1, int(y_coo["y1"])), (int(x_coo["x2"])+px1, int(y_coo["y2"])), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x2"]), int(y_coo["y2"])+px1), (int(x_coo["x3"]), int(y_coo["y3"])+px1), (25,25,200),3)
        
        # Drawing second lines on 2nd marker--> 2nd one: 20 meter
        cv2.line(frame,(int(x_coo["x0"]), int(y_coo["y0"])-(2*px1)),(int(x_coo["x1"]), int(y_coo["y1"])-(2*px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x0"])-(2*px1), int(y_coo["y0"])),(int(x_coo["x3"])-(2*px1), int(y_coo["y3"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x1"])+(2*px1), int(y_coo["y1"])),(int(x_coo["x2"])+(2*px1), int(y_coo["y2"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x2"]), int(y_coo["y2"])+(2*px1)),(int(x_coo["x3"]), int(y_coo["y3"])+(2*px1)),(25,25,200),3)
        
        # Drawing first lines on 2st marker--> 2st one: 20 meter
        cv2.line(frame,(int(x_coo["x4"]), int(y_coo["y4"])-(px1)),(int(x_coo["x5"]), int(y_coo["y5"])-(px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x4"])-(px1), int(y_coo["y4"])),(int(x_coo["x7"])-(px1), int(y_coo["y7"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x5"])+(px1), int(y_coo["y5"])),(int(x_coo["x6"])+(px1), int(y_coo["y6"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x6"]), int(y_coo["y6"])+(px1)),(int(x_coo["x7"]), int(y_coo["y7"])+(px1)),(25,25,200),3)
        
        # Drawing second lines on 2nd marker--> 2nd one: 20 meter
        cv2.line(frame,(int(x_coo["x4"]), int(y_coo["y4"])-(2*px1)),(int(x_coo["x5"]), int(y_coo["y5"])-(2*px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x4"])-(2*px1), int(y_coo["y4"])),(int(x_coo["x7"])-(2*px1), int(y_coo["y7"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x5"])+(2*px1), int(y_coo["y5"])),(int(x_coo["x6"])+(2*px1), int(y_coo["y6"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x6"]), int(y_coo["y6"])+(2*px1)),(int(x_coo["x7"]), int(y_coo["y7"])+(2*px1)),(25,25,200),3)
        
    if(len(ids) == 3):
        
        # Drawing lines --> 1st one: 10 meter 
        cv2.line(frame, (int(x_coo["x0"]), int(y_coo["y0"])-px1), (int(x_coo["x1"]), int(y_coo["y1"])-px1), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x0"])-px1, int(y_coo["y0"])), (int(x_coo["x3"])-px1, int(y_coo["y3"])), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x1"])+px1, int(y_coo["y1"])), (int(x_coo["x2"])+px1, int(y_coo["y2"])), (25,25,200),3)
        cv2.line(frame, (int(x_coo["x2"]), int(y_coo["y2"])+px1), (int(x_coo["x3"]), int(y_coo["y3"])+px1), (25,25,200),3)
        
        # Drawing lines --> 1st one: 20 meter
        cv2.line(frame,(int(x_coo["x0"]), int(y_coo["y0"])-(2*px1)),(int(x_coo["x1"]), int(y_coo["y1"])-(2*px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x0"])-(2*px1), int(y_coo["y0"])),(int(x_coo["x3"])-(2*px1), int(y_coo["y3"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x1"])+(2*px1), int(y_coo["y1"])),(int(x_coo["x2"])+(2*px1), int(y_coo["y2"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x2"]), int(y_coo["y2"])+(2*px1)),(int(x_coo["x3"]), int(y_coo["y3"])+(2*px1)),(25,25,200),3)
        
        # Drawing lines --> 1st one: 20 meter
        cv2.line(frame,(int(x_coo["x4"]), int(y_coo["y4"])-(px1)),(int(x_coo["x5"]), int(y_coo["y5"])-(px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x4"])-(px1), int(y_coo["y4"])),(int(x_coo["x7"])-(px1), int(y_coo["y7"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x5"])+(px1), int(y_coo["y5"])),(int(x_coo["x6"])+(px1), int(y_coo["y6"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x6"]), int(y_coo["y6"])+(px1)),(int(x_coo["x7"]), int(y_coo["y7"])+(px1)),(25,25,200),3)
        
        # Drawing lines --> 1st one: 20 meter
        cv2.line(frame,(int(x_coo["x4"]), int(y_coo["y4"])-(2*px1)),(int(x_coo["x5"]), int(y_coo["y5"])-(2*px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x4"])-(2*px1), int(y_coo["y4"])),(int(x_coo["x7"])-(2*px1), int(y_coo["y7"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x5"])+(2*px1), int(y_coo["y5"])),(int(x_coo["x6"])+(2*px1), int(y_coo["y6"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x6"]), int(y_coo["y6"])+(2*px1)),(int(x_coo["x7"]), int(y_coo["y7"])+(2*px1)),(25,25,200),3)
        
        # Drawing lines --> 1st one: 20 meter
        cv2.line(frame,(int(x_coo["x8"]), int(y_coo["y8"])-(px1)),(int(x_coo["x9"]), int(y_coo["y9"])-(px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x8"])-(px1), int(y_coo["y8"])),(int(x_coo["x11"])-(px1), int(y_coo["y11"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x9"])+(px1), int(y_coo["y9"])),(int(x_coo["x10"])+(px1), int(y_coo["y10"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x10"]), int(y_coo["y10"])+(px1)),(int(x_coo["x11"]), int(y_coo["y11"])+(px1)),(25,25,200),3)
        
        # Drawing lines --> 1st one: 20 meter
        cv2.line(frame,(int(x_coo["x8"]), int(y_coo["y8"])-(2*px1)),(int(x_coo["x9"]), int(y_coo["y9"])-(2*px1)),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x8"])-(2*px1), int(y_coo["y8"])),(int(x_coo["x11"])-(2*px1), int(y_coo["y11"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x9"])+(2*px1), int(y_coo["y9"])),(int(x_coo["x10"])+(2*px1), int(y_coo["y10"])),(25,25,200),3)
        cv2.line(frame,(int(x_coo["x10"]), int(y_coo["y10"])+(2*px1)),(int(x_coo["x11"]), int(y_coo["y11"])+(2*px1)),(25,25,200),3)
