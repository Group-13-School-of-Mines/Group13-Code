git#include "DualMC33926MotorShield.h"
#include <Wire.h>
#include <math.h>
#define TICKS_PER_ROTATION  3200  //50*64
#define FAIRRER             0.05    //Fair encoder error
#define DUTY_CYCLE          127
//@ no wheel/torque resistance FAIRRER should be a fairly high number, close to 85 for 127 Duty Cycle
//@ high wheel/torque resistance FAIRRER should be close to 0
#define WHEEL_RADIUS        .13  //meters
#define ROBOT_WIDTH         .029845/2   //centimeters
//.242782
//.246
//.14605
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

byte receiveData;
byte sendData;

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
    }}

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

  //Timer ISR
  TIMSK2 = (TIMSK2 & B11111110) | 0x01; //enable timer overflow
  TCCR2B = (TCCR2B & B11111010) | 0x05; //128:1 prescalar

  md.init();
}

void loop() {
  rotatebot(-90);
  delay(2000);
  rotatebot(90);
  delay(2000);
  //movebot(0.3048*2); //2 feet 
  
}

void movebot(float distance) {
  if(distance == 0) return;
  Serial.println("distance: ");
  Serial.println(distance);
  double startX = x;
  double startY = y;
  
  //forward
  if(distance > 0) {
    digitalWrite(MOTORDIR[0], false);
    digitalWrite(MOTORDIR[1], true);
    analogWrite(MOTORPWM[0], DUTY_CYCLE);
    analogWrite(MOTORPWM[1], DUTY_CYCLE);
  }
 if(distance < 0) {
    digitalWrite(MOTORDIR[0], true);
    digitalWrite(MOTORDIR[1], false);
    analogWrite(MOTORPWM[0], DUTY_CYCLE);
    analogWrite(MOTORPWM[1], DUTY_CYCLE);
    distance = -distance;
  }

  float traveled = sqrt(pow((startX - x), 2) + pow((startY - y), 2));
  float remaining = (distance - traveled);

  //speed based on distance
  while(remaining > FAIRRER) {
    traveled = sqrt(pow((startX - x), 2) + pow((startY - y), 2));
    remaining = (distance - traveled);
    
    int pwm = int(200*exp(-pow(distance/2 - traveled,2)/(pow(distance/2, 2)*2/3)))+10;
    analogWrite(MOTORPWM[0], pwm);
    analogWrite(MOTORPWM[1], pwm);
    Serial.println(pwm);
  }

  //stop motors after movement
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
    analogWrite(MOTORPWM[0], DUTY_CYCLE);
    analogWrite(MOTORPWM[1], DUTY_CYCLE);
    while(phi > rads+startPhi) {
      sendData = int(rotation1/16);
      //Serial.println(sendData);
      Serial.println(phi);
    }
  }
  
  //rotate left
  else if(rads > 0) {
    digitalWrite(MOTORDIR[0], true);
    digitalWrite(MOTORDIR[1], true);
    analogWrite(MOTORPWM[0], DUTY_CYCLE);
    analogWrite(MOTORPWM[1], DUTY_CYCLE);
    while(phi < rads + startPhi) {
      sendData = int(rotation1/16);
      //Serial.println(sendData);
      Serial.println(phi);
    }
  }
  //stop motors
  analogWrite(MOTORPWM[0], 0);
  analogWrite(MOTORPWM[1], 0);
}


ISR(TIMER2_OVF_vect) {
  TCNT2 = 30; //set timer count to 30 for a ~0.1s period
  DataRequest(1); //send position data
}

void DataReceive(int numBytes) {
  while(Wire.available()) {
    receiveData = Wire.read();
  }
}

//send data through i2c
void DataRequest(int numBytes) {
  Wire.write(sendData);
}

void turn() {
  //wheel 1
  encoder1last = encoder1current;
  encoder1current = digitalRead(ENCODER[0])*2 + digitalRead(ENCODER[1]);
  if (
    (encoder1last == 1 && encoder1current == 3) || 
    (encoder1last == 3 && encoder1current == 2) ||
    (encoder1last == 2 && encoder1current == 0) ||
    (encoder1last == 0 && encoder1current == 1)) {
      updateLocRot(0, -1);
      }
  else if (
    (encoder1last == 3 && encoder1current == 1) || 
    (encoder1last == 1 && encoder1current == 0) ||
    (encoder1last == 0 && encoder1current == 2) ||
    (encoder1last == 2 && encoder1current == 3)) {
      updateLocRot(0, 1);
      }

  //wheel 2
  encoder2last = encoder2current;
  encoder2current = digitalRead(ENCODER[2])*2 + digitalRead(ENCODER[3]);
  
  if (
    (encoder2last == 1 && encoder2current == 3) || 
    (encoder2last == 3 && encoder2current == 2) ||
    (encoder2last == 2 && encoder2current == 0) ||
    (encoder2last == 0 && encoder2current == 1)) {
      updateLocRot(1, 1);
      }
  else if (
    (encoder2last == 3 && encoder2current == 1) || 
    (encoder2last == 1 && encoder2current == 0) ||
    (encoder2last == 0 && encoder2current == 2) ||
    (encoder2last == 2 && encoder2current == 3)) {
      updateLocRot(1, -1);
      }
  //Serial.print(v1);
  //Serial.print("\t");
  //Serial.print(v2);
  //Serial.print("\t");
  //Serial.print(x);
  //Serial.print("\t");
  //Serial.print(y);
  //Serial.print("\t");
  //Serial.print(phi);
  //Serial.print("\n"); 
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
  
  //Serial.println("Wheel stopped!");
  analogWrite(MOTORPWM[0], 0);
}
