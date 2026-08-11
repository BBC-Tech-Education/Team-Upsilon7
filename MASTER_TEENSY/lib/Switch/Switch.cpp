#include "Switch.h"



void Switch::init(uint8_t pin){
    pin_value = pin;
    pinMode(pin_value, INPUT_PULLUP);

}



uint8_t Switch::read(){
    // reads switches value
    uint8_t Switch_value = digitalRead(pin_value);
    return !Switch_value;

}
