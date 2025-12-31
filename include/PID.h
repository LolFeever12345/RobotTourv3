#ifndef PID_H
#define PID_H
#include <Arduino.h>
#include <Encoder.h>
#include "Motor.h"

class PID{
    public:
        PID(float Kp, float Ki, float Kd);
        void output(Encoder& enc, Motor& motor, int speed);
        void test(Encoder& enc, Motor& motor);

        const float MPC = (PI*40)/900; // mm/count

    private:
        float Kp;
        float Ki;
        float Kd;

        unsigned long prevTime = 0;
        float integral = 0.0;
        float prev_error = 0.0;
        float prevDistance = 0.0;
};


#endif 