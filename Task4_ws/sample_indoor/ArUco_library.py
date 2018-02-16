############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
    ## function to detect ArUco markers in the image using ArUco library
    ## argument: img is the test image
    ## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    ## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    ## 				{0: array([[315, 163],
    #							[319, 263],
    #							[219, 267],
    #							[215,167]], dtype=float32)}

    Detected_ArUco_markers = {}
    ## enter your code here ##
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)

    parameters = aruco.DetectorParameters_create()

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    
    for i in xrange(len(corners)):
        Detected_ArUco_markers[ids[i][0]] = corners[i][0]

    return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
    ## function to calculate orientation of ArUco with respective to the scale mentioned in Problem_Statement.pdf
    ## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    ## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the ProblemStatement.pdf)
    ##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
    ##			function should return: {1: 120 , 2: 164}

    ArUco_marker_angles = {}
    ## enter your code here ##
    for i in Detected_ArUco_markers:
        points = Detected_ArUco_markers[i]
        top_left = points[0]
        top_right = points[1]
        bottom_right = points[2]
        origin = ((top_left[0]+bottom_right[0])/2.0, (top_left[1]+bottom_right[1])/2.0)    #ORIGIN IS CENTER OF MARKER
        center = ((top_left[0]+top_right[0])/2.0,(top_left[1]+top_right[1])/2.0)            #CENTER IS MID POINT OF TOP-LEFT POINT AND TOP-RIGHT POINT
        
        degrees = math.degrees(math.atan2(origin[1]-center[1], center[0]-origin[0]))

        ##correct the angle
        if degrees < 0:
            degrees = 360 + degrees
        ArUco_marker_angles[i] = int(round(degrees))
    

    return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
    ## function to mark ArUco in the test image as per the instructions given in problem_statement.pdf 
    ## arguments: img is the test image 
    ##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
    ##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
    ## return: image namely img after marking the aruco as per the instruction given in Problem_statement.pdf

    ## enter your code here ##
    for i in Detected_ArUco_markers:
        points = Detected_ArUco_markers[i]
        top_left = points[0]
        top_right = points[1]
        bottom_right = points[2]
        bottom_left = points[3]
        #color = [  Gray,        Green,       Pink,          White,        Red,        Blue]
        color = [(125,125,125), (0,255,0), (180,105,255), (255,255,255), (0,0,255), (255,0,0)]
        origin = (int(top_left[0]+bottom_right[0])//2, int(top_left[1]+bottom_right[1])//2)       #ORIGIN IS THE CENTER OF MARKER
        center = (int(top_left[0]+top_right[0])//2, int(top_left[1]+top_right[1])//2)               #CENTER IS MID POINT OF TOP-LEFT POINT AND TOP-RIGHT POINT
        
        for j in xrange(4):
            cv2.circle(img, tuple(points[j]), 5, color[j], -1)

        cv2.circle(img, (origin), 5, color[4], -1)
        cv2.line(img, (origin), (center), color[5], 3)

        id_coord = (origin[0]+20, origin[1])
        angle_coord = (origin[0]-80, origin[1])

        font = cv2.FONT_HERSHEY_SIMPLEX    
        cv2.putText(img, str(i), id_coord, font, 1, color[4], 2, cv2.LINE_AA)
        cv2.putText(img, str(ArUco_marker_angles[i]), angle_coord, font, 1, color[1], 2, cv2.LINE_AA)

    return img


