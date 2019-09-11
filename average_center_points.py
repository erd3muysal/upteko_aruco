# -*- coding: utf-8 -*-
"""
Created on Tue Sep 10 15:08:22 2019

@author: R. Erdem Uysal
company: Upteko
"""

def avg_center_points(ids, c_x, c_y):
    """
    This script is basically finds the average centroid of multiple ArUco markers from its corner coordinates.
    """
    if(len(ids) == 1):
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
