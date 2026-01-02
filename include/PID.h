#ifndef PID_H
#define PID_H
#include <Arduino.h>
#include <Encoder.h>
#include "Motor.h"

class PID{
    public:
        PID(float Kp, float Ki, float Kd);
        int output(Encoder& enc, int speed, float dt);
        void reset();

        const float MPC = (PI*40.f)/909.72; // mm/count
        float aSpeed;
    private:
        float Kp;
        float Ki;
        float Kd;

        long prevTime = 0;
        float integral = 0.f;
        float prev_error = 0.f;
        float prevDistance = 0.f;

};


#endif 