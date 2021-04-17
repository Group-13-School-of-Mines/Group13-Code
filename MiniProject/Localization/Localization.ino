#include "DualMC33926MotorShield.h"
#include <Wire.h>
#define TICKS_PER_ROT     3200  //50*64
#define FAIRRER           85    //Fair encoder error
#define DUTY_CYCLE        127
//@ no wheel/torque resistance FAIRRER should be a fairly high number, close to 85 for 127 Duty Cycle
//@ high wheel/torque resistance FAIRRER should be close to 0

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
  
  motorspin(desiredRotation1);
  //DataRequest(1);
  //Serial.println(sendData);

}

ISR(TIMER2_OVF_vect) {
  TCNT2 = 30; //set timer count to 30 for a ~0.1s period
  DataRequest(1); //send position data
}

void DataReceive(int numBytes) {
  while(Wire.available()) {
    receiveData = Wire.read();
  }
  switch(receiveData) {
    //NW
    case 1:
      desiredRotation1 = 7*TICKS_PER_ROT/8;
      Serial.println("NW");
      break;
    
    //NE
    case 2:
      desiredRotation1 = 5*TICKS_PER_ROT/8;
      Serial.println("NE");
      break;

    //SE
    case 3:
      desiredRotation1 = 3*TICKS_PER_ROT/8;
      Serial.println("SE");
      break;

    //SW
    case 4:
      desiredRotation1 = 1*TICKS_PER_ROT/8;
      Serial.println("SW");
      break;
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
      rotation1++; 
      }
  else if (
    (encoder1last == 3 && encoder1current == 1) || 
    (encoder1last == 1 && encoder1current == 0) ||
    (encoder1last == 0 && encoder1current == 2) ||
    (encoder1last == 2 && encoder1current == 3)) {
      rotation1--; 
      }

  //wheel 2
  encoder2last = encoder2current;
  encoder2current = digitalRead(ENCODER[2])*2 + digitalRead(ENCODER[3]);
  if (
    (encoder1last == 1 && encoder1current == 3) || 
    (encoder1last == 3 && encoder1current == 2) ||
    (encoder1last == 2 && encoder1current == 0) ||
    (encoder1last == 0 && encoder1current == 1)) {
      rotation2++; 
      }
  else if (
    (encoder1last == 3 && encoder1current == 1) || 
    (encoder1last == 1 && encoder1current == 0) ||
    (encoder1last == 0 && encoder1current == 2) ||
    (encoder1last == 2 && encoder1current == 3)) {
      rotation2--; 
      }
}

void motorspin(int desiredRotation) {
  stopIfFault();
  if(desiredRotation > rotation1) {
    digitalWrite(MOTORDIR[0], false);
    analogWrite(MOTORPWM[0], DUTY_CYCLE);
    while(rotation1 < desiredRotation - FAIRRER) {
      sendData = int(rotation1 / 16);
      Serial.println(sendData);
      stopIfFault();
    }
  }
  else if(desiredRotation < rotation1) {
    digitalWrite(MOTORDIR[0], true);
    analogWrite(MOTORPWM[0], DUTY_CYCLE);
    while(rotation1 > desiredRotation + FAIRRER) {
      sendData = int(rotation1 / 16);
      Serial.println(sendData);
      stopIfFault();
    }
  }
  
  //Serial.println("Wheel stopped!");
  analogWrite(MOTORPWM[0], 0);
}
