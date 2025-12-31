#include "Motor.h"
#include "PID.h"
#include <Encoder.h>
#include <Arduino.h>

PID::PID(float Kp, float Ki, float Kd){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::test(Encoder& encoder, Motor& motor){
    motor.forward(255);
    Serial.println(encoder.read());
}

void PID::output(Encoder& enc, Motor& motor, int speed){
    unsigned long now = millis();
    float dt = (now-prevTime)/1000;
    if(dt<0.001f)return;
    prevTime = now;

    float currDistance = enc.read()*MPC;
    float deltaDistance = currDistance - prevDistance;
    prevDistance = currDistance;
    float currSpeed = deltaDistance/dt;
    float error = currSpeed - speed*1.0;

    integral += error*dt;

    float derivative = (error - prev_error)/dt;
    prev_error = error;

    float output = Kp*error + Ki*integral + Kd*derivative;
    output = constrain(output, -255.f, 255.f);

    if(output<0){
        motor.backward((uint8_t)abs(output));
    }else{
        motor.forward((uint8_t)(output));
    }

    Serial.print(now);Serial.print(",");Serial.print(currSpeed);Serial.print(",");Serial.println(output);

}
