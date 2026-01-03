#include <Arduino.h>
#include "Motor.h"
#include "PID.h"
#include "Drive.h"

Encoder Rencoder(3 /*Encoder Pin A*/, 5 /*Encoder Pin B*/);
Motor Rmotor(8 /*IN1*/, 9 /*IN2*/, 11 /*ENA*/);
Encoder Lencoder(2 /*Encoder Pin A*/, 4 /*Encoder Pin B*/);
Motor Lmotor(6 /*IN3*/ , 7 /*IN4*/ ,10 /*ENB*/);
PID right(0.1f/*Kp*/, 0.999f/*Ki*/, 0.f/*Kd*/, 40.f/*Diameter of the wheels*/, 75.81f /*Gear Ratio of Motor*/, 12 /*CPR*/);
PID left(0.1f/*Kp*/, 1.03f/*Ki*/, 0.f/*Kd*/, 40.f/*Diameter of the wheels*/, 75.81f /*Gear Ratio of Motor*/, 12 /*CPR*/);
uint8_t buttonState;

Drive drive(left, right, Lmotor, Rmotor, Lencoder, Rencoder, 106 /* The distance between the wheels*/, 40 /* The diameter of the wheels*/);


void setup() {
  Serial.begin(9600);
  Rmotor.begin();
  Lmotor.begin();
  pinMode(12, INPUT);
}

void loop() {
  buttonState = digitalRead(12);
  bool runState = false;
  if(buttonState == HIGH){
    runState = true;
    Serial.print("button pressed");
  }else{
    runState = false;
  }

  if(runState){
    drive.turn(45,100);
    delay(5000);
    drive.stop();
    while(1);
    runState = false;
  }
}

