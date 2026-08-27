#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <math.h>
#include <Arduino.h>
#include <Servo.h>

class Servo_Motor {
public:
    Servo_Motor();
    int8_t attach(int8_t pin, bool direction);
    int8_t attach(int8_t pin, bool direction, int8_t min_angle, int16_t max_angle, int16_t min_us, int16_t max_us);
    void detach();
    void write_speed(int16_t speed);
    void write_speed_float(float speed);
    void write_angle(int16_t angle);
    void write_microseconds(int16_t value);
    int16_t read_speed();
    int16_t read_angle();
    int16_t read_microseconds();
    bool attached(); 

private:
    int servo_pin;
    bool servo_direction;

    int16_t pulse_width = 1500;
    int8_t minimum_angle = 0;
    int16_t maximum_angle = 180;
    int16_t minimum_us = 1000;
    int16_t maximum_us = 2000;

    bool servo_attached = false;

    Servo myservo;
};
#endif