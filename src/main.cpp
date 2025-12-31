#include <Arduino.h>
#include "Motor.h"
#include "PID.h"

Encoder encoder(3,5);
Motor motor(8,9,11);
PID right(1.0, 0.0, 0.0);


void setup() {
  motor.begin();
}

void loop() {
  unsigned long timer = millis();
  if(timer < 10000){
    right.output(encoder, motor, 400);
  }else{
    motor.forward(0);
  }
}

