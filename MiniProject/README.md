# Mini Project

This section is dedicated to the Mini project of EENG350
Build a robot that correctly identifies the quadrant of an aruco marker and position the wheel accordingly

The Computer Vision (Nonintegrated) code detetcts which quadrant the marker is in then prints it.

The Computer Vision (Integrated) code detects which quadrant the marker is in, sends that to the arduino, reads where the wheel is from the arduino, then sends that to the LCDscreen.

The Localization code waits for input from the Raspberry pi, moves to that location, while sending the location to the pi at regular intervals
