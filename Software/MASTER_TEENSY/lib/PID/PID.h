#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    PID(float p, float i, float d, float absoluteMax = 0.0);
    float update(float input, float setpoint);
private:
    float kp;
    float ki;
    float kd;
    uint32_t lastTime;
    float lastError = 0;
    float absMax;
    float integral;
};

#endif