import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import smbus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd



# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

actual = 0
var  = 0
# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)


# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    bus.write_byte(address, value)
    #bus.write_byte_data(address, 0, value)
    return -1
def readNumber():
    number = bus.read_byte(address)
    
    #number = bus.read_byte_data(address, 1)
    return number


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
        
                if center[0] < gray.shape[0]*.5:
                    if center[1]+30 < gray.shape[1]*.5:
                        print("NW")
                        var = 1
                        writeNumber(var)
                        lcd.color = [0, 100, 0]
                        time.sleep(.1)
                        actual = readNumber()
                        actual = (actual/200)*2*3.14159
                        lcd.message = "Sent: " + var + "\nGot: " + actual 
                    else:
                        print("SW")
                        var = 4
                        writeNumber(var)
                        lcd.color = [0, 100, 0]
                        time.sleep(.1)
                        actual = readNumber()
                        actual = (actual/200)*2*3.14159
                        lcd.message = "Sent: " + var + "\nGot: " + actual
                else:
                    if center[1]+30 < gray.shape[1]*.5:
                        print("NE")
                        var = 2
                        writeNumber(var)
                        lcd.color = [0, 100, 0]
                        time.sleep(.1)
                        actual = readNumber()
                        actual = (actual/200)*2*3.14159
                        lcd.message = "Sent: " + var + "\nGot: " + actual
                    else:
                        print("SE")
                        var = 3
                        writeNumber(var)
                        lcd.color = [0, 100, 0]
                        time.sleep(.1)
                        actual = readNumber()
                        actual = (actual/200)*2*3.14159
                        lcd.message = "Sent: " + var + "\nGot: " + actual
            
        
   
