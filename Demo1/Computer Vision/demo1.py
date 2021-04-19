#!/usr/bin/env python3
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
def demo1():
    cap = cv2.VideoCapture(0)
    while True:
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Video',gray)
        cv2.waitKey(1)
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict,parameters=arucoParams)
        if len(corners) > 0: 
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                height = (bottomLeft[1]-topLeft[1]+bottomRight[1]-topRight[1])/2
                if height == 0:
                    height = 1
                width = (topRight[0]-topLeft[0]+bottomRight[0]-bottomLeft[0])/2
                a = (8.5*620)/height #Distance formula
                center = (bottomRight[0]-((bottomRight[0]-bottomLeft[0])/2),bottomLeft[1]-((bottomLeft[1]-topLeft[1])/2))
                angle =((center[0]-320)/320)*26
                distance = a/math.cos(math.radians(abs(angle)))
                print('Distance from camera: ',distance)
                #angle = abs((1-(width/height)*90) #Angle formula
                print('Angle: ',angle)
                
demo1()
