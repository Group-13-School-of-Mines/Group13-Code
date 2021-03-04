import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
def mini_project():
    cap = cv2.VideoCapture(0)
    while True:
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow('Video',gray)
        cv2.waitKey(1)
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
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
                center = (topRight[0]-((topRight[0]-topLeft[0])/2),bottomLeft[1]-((bottomLeft[1]-topLeft[1])/2))
                print(center)
                if center[0] < gray.shape[0]*.5:
                    if center[1]+30 < gray.shape[1]*.5:
                        print("NW")
                    else:
                        print("SW")
                else:
                    if center[1]+30 < gray.shape[1]*.5:
                        print("NE")
                    else:
                        print("SE")
        cv2.ddestroyAllWindows()
