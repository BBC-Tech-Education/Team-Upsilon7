#include "PID.h"

PID::PID(float p, float i, float d, float absoluteMax) {
    kp = p;
    ki = i;
    kd = d;
    absMax = absoluteMax;

    integral = 0.0f;
    lastError = 0.0f;
    lastTime = micros();
}

float PID::update(float input, float setpoint) {
    float derivative = 0.0f; // Default to 0.0 if no time has passed
    float error = setpoint - input;

    uint32_t currentTime = micros();
    float elapsedTime = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;

    integral += elapsedTime * error;

    derivative = -(error - lastError) / elapsedTime;
    
    lastError = error;
    lastTime = currentTime;

    float correction = kp * error + ki * integral - kd * derivative;

    return absMax == 0.0f ? correction : constrain(correction, -absMax, absMax);
}