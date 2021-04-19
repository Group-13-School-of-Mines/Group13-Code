#!/usr/bin/env python3

import os
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import math
import time
import smbus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
bus = smbus.SMBus(1)

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
    os.system('stty -F /dev/ttyACM0 raw 9600')
    cap = cv2.VideoCapture(0)

    while True:
        sendRotate(15)
        _, frame = cap.read()
        if frame.all() == None:
            raise Exception("Could not load video!")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
       ## cv2.imshow('Video',gray)
       ## cv2.waitKey(1)

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
                if angle <=15 and angle >= -15:
                    print("sending stop")
                    sendStop()

                distance = a/math.cos(math.radians(abs(angle)))
                dist = int(distance*.0328)
                #writeNumber(dist)
                print("sending move")
                sendMove(dist)

                print('Distance from camera: ',distance)
                #angle = abs((1-(width/height)*90) #Angle formula
                print('Angle: ',angle)

demo2()
