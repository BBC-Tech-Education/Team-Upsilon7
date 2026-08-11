#ifndef SWITCH_H
#define SWITCH_H

#include <Config.h>
#include <Arduino.h>
#include <math.h>


class Switch {
public:
    Switch() {}
    void init(uint8_t pin);
    uint8_t read();

private:
    uint8_t pin_value;

};
#endif