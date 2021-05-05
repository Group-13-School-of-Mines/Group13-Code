#!/usr/bin/env python3
import os
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import math
import time

import imutils
from imutils.video import VideoStream

#Arduino commands
def sendStop():
    os.system('echo \~ > /dev/ttyACM0')
def sendRotate(degrees):
    if(len(str(degrees)) > 4):
        print("Argument too long. must be 3 digits at most")
    else:
        os.system('echo ,' + str(degrees) + '! > /dev/ttyACM0')
def sendMove(feet):
    if(len(str(feet)) > 4):
        print("Argument too long. must be 3 digits at most")
    else:
        os.system('echo :' + str(feet) + '! > /dev/ttyACM0')
def sendCircle(radius):
    if(len(str(radius)) > 4):
        print("Argument too long. must be 3 digits at most")
    else:
        os.system('echo \;' + str(radius) + '! > /dev/ttyACM0')

# Main function
def demo2():
    usingPiCamera = True
    frameSize = (640, 480)
    cap = VideoStream(src = 0, usePiCamera = usingPiCamera, resolution = frameSize, framerate = 32).start()
    time.sleep(1.0)

    os.system('stty -F /dev/ttyACM0 raw 9600')
    #sendCircle(0)
    angle = 180
    seeIt = False
    #send a big rotate so it doesnt get stuck
    sendRotate(45)
    time.sleep(0.1)
    while (seeIt == False):
        sendRotate(5)
        time.sleep(0.1)
        frame = cap.read()
        #cv2.imshow('Video',frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict,parameters=arucoParams)

        if len(corners) > 0:
          seeIt = True
    seeIt = False
    while (seeIt == False):
        frame = cap.read()
        #cv2.imshow('Video',frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
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
                a = (11*620)/height #Distance formula
                center = (bottomRight[0]-((bottomRight[0]-bottomLeft[0])/2),bottomLeft[1]-((bottomLeft[1]-topLeft[1])/2))
                angle =((center[0]-320)/320)*26
                sendRotate(-angle)
                sendStop()
                seeIt = True

    seeIt = False
    while (seeIt == False):
        sendMove(1)
        time.sleep(0.1)
        frame = cap.read()
        #cv2.imshow('Video',frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
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
                a = (11*620)/height #Distance formula
                center = (bottomRight[0]-((bottomRight[0]-bottomLeft[0])/2),bottomLeft[1]-((bottomLeft[1]-topLeft[1])/2))
                angle =((center[0]-320)/320)*26
                distance = abs(a/math.cos(math.radians(abs(angle))))
                dist = int(distance*.0328)
                if dist <= 1.8 or len(corners) <= 0:
                    print("sending stop")
                    sendStop()
                    seeIt = True
                elif len(corners) <= 0:
                    print("cant see it")
                    sendMove(1)
                    time.sleep(0.5)
                    sendStop()
                    seeIt = True
    time.sleep(1)
    #sendRotate(-90)
    time.sleep(2)
    #sendCircle(2)
    print('Distance from camera: ',distance)
    #angle = abs((1-(width/height)*90) #Angle formula
    print('Angle: ',angle)
demo2()
