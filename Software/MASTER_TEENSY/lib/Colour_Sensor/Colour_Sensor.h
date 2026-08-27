#ifndef COLOUR_SENSOR_H
#define COLOUR_SENSOR_H

#include <Arduino.h>
#include <Adafruit_AS7341.h>
#include "Config.h"

class Colour_Sensor {
public:
    Colour_Sensor() {}
    void init();
    void update();
    uint8_t see_colour(uint8_t colour);
    void calibrate(uint8_t colour);

private:
    bool is_reading = false;
    struct colour_values {
        u_int16_t Channel_415nm;
        u_int16_t Channel_445nm;
        u_int16_t Channel_480nm;
        u_int16_t Channel_515nm;
        u_int16_t Channel_555nm;
        u_int16_t Channel_590nm;
        u_int16_t Channel_630nm;
        u_int16_t Channel_680nm;
        u_int16_t Channel_Clear;
        u_int16_t Channel_Near_IR;
    };

    colour_values value;
    uint16_t thresholds[COLOUR_NUM] = {0};

    Adafruit_AS7341 as7341;
};


#endif