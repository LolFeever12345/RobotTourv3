#include "Drive.h"
#include <math.h>

void Drive::driveDistance(float distance, int speed){
    // int PWM = startPWM(speed);
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistance = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    // Lmotor.drive(PWM);
    // Rmotor.drive(PWM);
    while(abs(currDistance)<abs(distance)){
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.1f){
            Lmotor.drive(Lcon.output(Lenc,speed,dt));
            Rmotor.drive(Rcon.output(Renc,speed,dt));
            prevTime = now;
            currDistance = (Lenc.read()*Lcon.MPC+Renc.read()*Rcon.MPC)/2.f;
            Serial.print((now-start)/1000.f);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.println(currDistance);
        }
        now = millis();
    }
    stop();
}

void Drive::stop(){
    Rmotor.drive(0);
    Lmotor.drive(0);
}

void Drive::reset(){
    Renc.write(0);
    Lenc.write(0);
}

uint8_t Drive::startPWM(int linSpeed){
    float outputI = 40.f-40.f*log(1-abs(linSpeed)/550.f);
    uint8_t output = constrain((int)outputI, 40,230);
    return output;
}

void Drive::turnL(int speed){
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistanceLeft = 0.f;
    float currDistanceRight = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    while(currDistanceLeft < (250+wBase/2.f)*(PI/2.f) && currDistanceRight < (250-wBase/2.f)*(PI/2.f)){
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.1f){
            Lmotor.drive(Lcon.output(Lenc,speed*1.54,dt));
            Rmotor.drive(Rcon.output(Renc,speed,dt));
            prevTime = now;
            currDistanceLeft = Lenc.read()*Lcon.MPC;
            currDistanceRight = Renc.read()*Rcon.MPC;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed*1.54f);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(currDistanceLeft);Serial.print(",");
            Serial.println(currDistanceRight);
        }
        now = millis();
    }
    stop();
}

void Drive::turnR(int speed){
    Serial.println((250-wBase/2.f)*(PI/2.f));
    Serial.println((250+wBase/2.f)*(PI/2.f));
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistanceLeft = 0.f;
    float currDistanceRight = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    while(currDistanceLeft < (250-wBase/2.f)*(PI/2.f) && currDistanceRight < (250+wBase/2.f)*(PI/2.f)){
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.1f){
            Lmotor.drive(Lcon.output(Lenc,speed*1.54f,dt));
            Rmotor.drive(Rcon.output(Renc,speed,dt));
            prevTime = now;
            currDistanceLeft = Lenc.read()*Lcon.MPC;
            currDistanceRight = Renc.read()*Rcon.MPC;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(speed*1.54f);Serial.print(",");
            Serial.print(currDistanceLeft);Serial.print(",");
            Serial.println(currDistanceRight);
        }
        now = millis();
    }
    stop();
}

void Drive::sTurnL(int speed){
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistanceLeft = 0.f;
    float currDistanceRight = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    while(abs(currDistanceLeft) < (wBase*PI)/4.f && abs(currDistanceRight) < (wBase*PI)/4.f){
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.1f){
            Lmotor.drive(Lcon.output(Lenc,speed*-1,dt));
            Rmotor.drive(Rcon.output(Renc,speed,dt));
            prevTime = now;
            currDistanceLeft = Lenc.read()*Lcon.MPC;
            currDistanceRight = Renc.read()*Rcon.MPC;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed*-1);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(currDistanceLeft);Serial.print(",");
            Serial.println(currDistanceRight);
        }
        now = millis();
    }
    stop();
}

void Drive::sTurnR(int speed){
    reset();
    Lcon.reset();
    Rcon.reset();
    unsigned long prevTime = millis();
    float currDistanceLeft = 0.f;
    float currDistanceRight = 0.f;
    unsigned long now = millis();
    unsigned long start = millis();
    float dt = (now-prevTime)/1000.f;
    while(abs(currDistanceLeft) < (wBase*PI)/4.f && abs(currDistanceRight) < (wBase*PI)/4.f){
        dt = (now-prevTime)/1000.f;
        if(dt>= 0.1f){
            Lmotor.drive(Lcon.output(Lenc,speed,dt));
            Rmotor.drive(Rcon.output(Renc,speed*-1,dt));
            prevTime = now;
            currDistanceLeft = Lenc.read()*Lcon.MPC;
            currDistanceRight = Renc.read()*Rcon.MPC;
            Serial.print(now-start);Serial.print(",");
            Serial.print(Lcon.aSpeed);Serial.print(",");
            Serial.print(Rcon.aSpeed);Serial.print(",");
            Serial.print(speed);Serial.print(",");
            Serial.print(speed*-1);Serial.print(",");
            Serial.print(currDistanceLeft);Serial.print(",");
            Serial.println(currDistanceRight);
        }
        now = millis();
    }
    stop();
}


