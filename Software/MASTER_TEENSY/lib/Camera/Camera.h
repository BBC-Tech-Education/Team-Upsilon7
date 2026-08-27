#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <math.h>

#define START_BYTE 0xAA
#define END_BYTE   254

class CAMERA {
public:
    CAMERA(HardwareSerialIMXRT* serial);
    void init();
    uint8_t read_camera();

private:
    HardwareSerialIMXRT* openMV; 
    uint8_t camera_value;

};
#endif