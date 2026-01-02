#include <Arduino.h>
#include "Motor.h"
#include "PID.h"
#include "Drive.h"

Encoder Rencoder(3,5);
Motor Rmotor(8,9,11);
Encoder Lencoder(2,4);
Motor Lmotor(6,7,10);
PID right(0.1, 0.999, 0.0);
PID left(0.1, 1.03, 0.0);
uint8_t buttonState;

Drive drive(left, right, Lmotor, Rmotor, Lencoder, Rencoder);


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
    drive.driveDistance(500,100);
    drive.turnR(100);
    drive.driveDistance(500,100);
    delay(5000);
    drive.stop();
    while(1);
    runState = false;
  }
}

