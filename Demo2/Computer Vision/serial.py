#!/usr/bin/env python3

import os

#Arduino commands
os.system('stty -F /dev/ttyACM0 raw 9600')

def sendStop():
    os.system('echo \~ > /dev/ttyACM0')

def sendRotate(degrees):
    if(len(str(degrees)) > 3):
        print("Argument too long. must be 3 digits at most")
    else:
        os.system('echo ,' + str(degrees) + '! > /dev/ttyACM0')

def sendMove(feet):
    if(len(str(feet)) > 3):
        print("Argument too long. must be 3 digits at most")
    else:
        os.system('echo :' + str(feet) + '! > /dev/ttyACM0')

def sendCircle(radius):
    if(len(str(radius)) > 3):
        print("Argument too long. must be 3 digits at most")
    else:
        os.system('echo \;' + str(radius) + '! > /dev/ttyACM0')

sendCircle(0.1)
