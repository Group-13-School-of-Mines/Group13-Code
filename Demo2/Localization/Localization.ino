/*
* 00  stop
* 01  move (0 for infinite)
* 10  rotate
* 11  circle
*/

#include "DualMC33926MotorShield.h"
#include <Wire.h>
#include <math.h>
#define TICKS_PER_ROTATION  3200  //50*64
#define DUTY_CYCLE          127
#define WHEEL_RADIUS        0.084
#define ROBOT_WIDTH         0.027
DualMC33926MotorShield md;

//Pin assignments
const int ENCODER[]     = {6, 5, 13, 11};   //1A,B; 2A,B
const int ENCODER_ISR[] = {2, 3};           //1,2
const int MOTORDIR[]    = {7, 8};           //1,2
const int MOTORPWM[]    = {9, 10};          //1,2

//Global variables
int desiredRotation1 = 0;
int desiredRotation2 = 0;
int rotation1 = 0;
int rotation2 = 0;

int pwm = 0;
int minpwm = 65;

byte receiveData;
byte sendData;
bool stopCommand;

int encoder1last = 0;
int encoder1current = 0;
int encoder2last = 0;
int encoder2current = 0;

int lastTime1 = 0;
int currentTime1 = 0;
int lastTime2 = 0;
int currentTime2 = 0;

float deltaT1 = 0;
float deltaT2 = 0;

double v1 = 0;
double v2 = 0;

double x = 0;
double y = 0;
double phi = 0;

void stopIfFault() {
    if (md.getFault()) {
        Serial.println("Motor Controller fault");
        while(true);
    }
}

void setup() {
    //set up debug console
    Serial.begin(115200);
    Serial.println("Debug console:");

    //Join I2C bus as a slave with address 04
    Wire.begin(0x04);
    Wire.onReceive(DataReceive);
    Wire.onRequest(DataRequest);

    //Encoder pins
    pinMode(ENCODER[0], INPUT);
    pinMode(ENCODER[1], INPUT);
    pinMode(ENCODER[2], INPUT);
    pinMode(ENCODER[3], INPUT);
    pinMode(ENCODER_ISR[0], INPUT);
    pinMode(ENCODER_ISR[1], INPUT);

    //Motor controller pins
    pinMode(MOTORDIR[0], OUTPUT);
    pinMode(MOTORDIR[1], OUTPUT);
    pinMode(MOTORPWM[0], OUTPUT);
    pinMode(MOTORPWM[1], OUTPUT);

    //Interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_ISR[0]), turn, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_ISR[1]), turn, CHANGE);

    md.init();
}

void loop() {

    //circlebot(0);
    //wait before looping    
    delay(1000);
}

void movebot(float distance) {
    if(distance == 0) return;
    distance *= 0.3048;
    Serial.println("distance: ");
    Serial.println(distance);
    double startX = x;
    double startY = y;
  
    //forward
    if(distance > 0) {
        digitalWrite(MOTORDIR[0], false);
        digitalWrite(MOTORDIR[1], true);
    }
    //backward
    if(distance < 0) {
        digitalWrite(MOTORDIR[0], true);
        digitalWrite(MOTORDIR[1], false);
        distance = -distance;
    }

    float traveled = sqrt(pow((startX - x), 2) + pow((startY - y), 2));
    while(traveled < distance) {
        traveled = sqrt(pow((startX - x), 2) + pow((startY - y), 2));
        if (stopCommand) break; 
        pwm = int((
            pow(tanh(distance), 1) * 
            (255 - minpwm) * 
            exp(-pow(distance/2 - traveled,2) / (pow(distance/2, 2)*2/3))) + 
            minpwm);
    
        analogWrite(MOTORPWM[0], pwm);
        analogWrite(MOTORPWM[1], pwm);
        Serial.println(pwm);
    }

    //stop motors after movement
    stopCommand = false;
    analogWrite(MOTORPWM[0], false);
    analogWrite(MOTORPWM[1], false);
}

void rotatebot(float degrees) {
    float rads = degrees * 2*PI/360;
    Serial.println("destination:");
    Serial.println(rads);
    double startPhi = phi;
  
    //rotate right
    if (rads < 0) {
        digitalWrite(MOTORDIR[0], false);
        digitalWrite(MOTORDIR[1], false);
        rads = -rads - 0.05;
    }
    //rotate left
    else {
        digitalWrite(MOTORDIR[0], true);
        digitalWrite(MOTORDIR[1], true);
    }
    while(abs(phi - startPhi) < rads) {
        if (stopCommand) break; 
            pwm = int((
                pow(tanh(rads), 2) * 
                (175 - minpwm) * 
                exp(-pow(rads/2 - abs(phi-startPhi),2) / (pow(rads/2, 2)*2/3))) + 
                minpwm);
            analogWrite(MOTORPWM[0], pwm);
            analogWrite(MOTORPWM[1], pwm);    
            Serial.println(pwm);
    }
  
    //stop motors
    stopCommand = false;
    analogWrite(MOTORPWM[0], 0);
    analogWrite(MOTORPWM[1], 0);
}

void circlebot(float radius) {
    if(radius == 0) rotatebot(360);
//find maximum pwms for each wheel based on radius
    float leftRadius = radius*0.3048 - ROBOT_WIDTH*5;
    float rightRadius = radius*0.3048 + ROBOT_WIDTH*5;
    int multiplier = 500/radius;
    int leftpwm = int(leftRadius * multiplier);
    int rightpwm = int(rightRadius * multiplier);

    double startPhi = phi;

    if(leftpwm > 0) digitalWrite(MOTORDIR[0], false);
    if(leftpwm < 0) digitalWrite(MOTORDIR[0], true);
    if(rightpwm > 0) digitalWrite(MOTORDIR[1], true);
    if(rightpwm < 0) digitalWrite(MOTORDIR[1], false);
    analogWrite(MOTORPWM[0], abs(leftpwm));
    analogWrite(MOTORPWM[1], abs(rightpwm));     

    while(abs(phi-startPhi) < 2*PI*1.025) {
        if (stopCommand) break; 
        if(phi-startPhi < (2*PI*1.025)/24) {
            analogWrite(MOTORPWM[0], 0); 
            analogWrite(MOTORPWM[1], abs(rightpwm)); 
        }
        else {
            analogWrite(MOTORPWM[0], abs(leftpwm)); 
            analogWrite(MOTORPWM[1], abs(rightpwm)); 
        }
        Serial.println(phi-startPhi);
    }
   
    //stop motors
    stopCommand = false;
    analogWrite(MOTORPWM[0], 0);
    analogWrite(MOTORPWM[1], 0);
}


void DataReceive(int numBytes) {
    while(Wire.available()) {
        receiveData = Wire.read();
        Serial.print(receiveData, BIN);
        switch ((receiveData >> 6) & 0x03) {
            
            //stop
            case 0:
                stopCommand = true;
                break;

            //move
            case 1:
                //for infinite movement pass 0
                if(receiveData & 0x3F == 0x00) {
                    digitalWrite(MOTORDIR[0], false);
                    digitalWrite(MOTORDIR[1], true);
                    analogWrite(MOTORPWM[0], 100);
                    analogWrite(MOTORPWM[1], 100);
                }
                //any other value returns a finite movement
                else{
                    movebot(int(receiveData & 0x3F));
                    delay(10);
                }
                break;

            //rotate
            case 2:
                rotatebot(int(receiveData & 0x3F));
                delay(10);
                break;
            
            //circle
            case 3:
                circlebot(int(receiveData & 0x3F));
                delay(10);
                break;
        }
    }
}

//send data through i2c
void DataRequest(int numBytes) {
    Wire.write(sendData);
    //Wire.write(0x00);
}

void turn() {
    //wheel 1
    encoder1last = encoder1current;
    encoder1current = digitalRead(ENCODER[0])*2 + digitalRead(ENCODER[1]);
    if (encoder1last == 3 && encoder1current == 2) {
        updateLocRot(0, -4);
    }
    else if (encoder1last == 2 && encoder1current == 3) {
        updateLocRot(0, 4);
    }

    //wheel 2
    encoder2last = encoder2current;
    encoder2current = digitalRead(ENCODER[2])*2 + digitalRead(ENCODER[3]);
  
    if (encoder2last == 3 && encoder2current == 2) {
        updateLocRot(1, 4);
    }
    else if (encoder2last == 2 && encoder2current == 3) {
        updateLocRot(1, -4);
    }
}

void updateLocRot(bool wheel, int dir) {
    //Left
    if(!wheel) {
        rotation1 += dir;
        lastTime1 = currentTime1;
        currentTime1 = micros();
        deltaT1 = currentTime1 - lastTime1;
        if (deltaT1 == 0) deltaT1 = 0.5;
        v1 = WHEEL_RADIUS*dir*(2*PI/TICKS_PER_ROTATION) / ((float)deltaT1);
        x = x + cos(phi)*(v1*deltaT1)/2;
        y = y + sin(phi)*(v1*deltaT1)/2;
        phi = phi + (WHEEL_RADIUS/ROBOT_WIDTH)*(v1*deltaT1);
    }
  
    //Right
    else {
        rotation2 += dir;
        lastTime2 = currentTime2;
        currentTime2 = micros();
        deltaT2 = currentTime2 - lastTime2;
        if (deltaT2 == 0) deltaT2 = 0.5;
        v2 = WHEEL_RADIUS*dir*(2*PI/TICKS_PER_ROTATION) / ((float)deltaT2);
        x = x + cos(phi)*(-v2*deltaT2)/2;
        y = y + sin(phi)*(-v2*deltaT2)/2;
        phi = phi - (WHEEL_RADIUS/ROBOT_WIDTH)*(-v2*deltaT2);
    }
}
