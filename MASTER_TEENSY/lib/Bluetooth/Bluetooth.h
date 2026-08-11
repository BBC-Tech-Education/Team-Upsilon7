#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <math.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

class Bluetooth_Module {
public:
    Bluetooth_Module(uint8_t RX_Pin, uint8_t TX_Pin);
    void init();
    u_int16_t read();
    void write(u_int16_t value);

private:
    SoftwareSerial BT_Serial;

};
#endif