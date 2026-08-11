#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

class IMU {
public:
    IMU() {}
    void init();
    void read();

    float x_bearing_180();
    float y_bearing_180();
    float z_bearing_180();

    void set_target_bearing(int16_t bearing_change);
    float x_angle_diff();

private:
    void calibrate();
    float bearing_mod(float bearing);

    Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire1);

    sensors_event_t event;
    uint16_t target_bearing = 0;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    float initial_x = 0.0f;
    float initial_y = 0.0f;
    float initial_z = 0.0f;
};
#endif