#ifndef DRIVE_H
#define DRIVE_H
#include <Arduino.h>
#include <Encoder.h>
#include "PID.h"

class Drive{
    public:
        
        Drive(PID& Lcon, PID& Rcon, Motor& Lmotor,Motor& Rmotor, Encoder& Lenc, Encoder& Renc, float wheelBase, float wheelDiameter): 
            Lcon(Lcon),
            Rcon(Rcon),
            Rmotor(Rmotor),
            Lmotor(Lmotor),
            Lenc(Lenc),
            Renc(Renc)
    {
        wBase = wheelBase;
        wDiameter = wheelDiameter;
    }
        void driveDistance(float distance, int speed);
        void stop();
        void reset();
        void turnR(int speed);
        void turnL(int speed);
        void sTurnR(int speed);
        void sTurnL(int speed);

    private:

        PID& Lcon;
        PID& Rcon;
        Motor& Rmotor;
        Motor& Lmotor;
        Encoder& Lenc;
        Encoder& Renc;


        unsigned long now;
        float wBase;
        float wDiameter;

        uint8_t startPWM(int linSpeed);

};

#endif