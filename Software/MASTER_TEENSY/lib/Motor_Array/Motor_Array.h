#ifndef MOTOR_ARRAY_H
#define MOTOR_ARRAY_H

#include <math.h>
#include <Servo_Motor.h>
#include <Arduino.h>


class Motor_Array {
public:
    Motor_Array();
    void init(uint8_t pin_lf, uint8_t pin_lb, uint8_t pin_rf, uint8_t pin_rb);
    void move(int16_t Speed_Value_Left, int16_t Speed_Value_Right);
    void move_float(float Speed_Value_Left, float Speed_Value_Right);
    int16_t current_speed();

private:
    int16_t Speed_Value_Left;
    int16_t Speed_Value_Right;

    Servo_Motor motor_lf;
    Servo_Motor motor_lb;
    Servo_Motor motor_rf;
    Servo_Motor motor_rb;
};
#endif