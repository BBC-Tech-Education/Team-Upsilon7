#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <math.h>
#include <Config.h>

#define START_BYTE 0xAA
#define END_BYTE   254

class CAMERA {
public:
    CAMERA(HardwareSerialIMXRT* serial);
    void init();
    uint8_t read_camera();
    uint8_t process_data(uint16_t sensor_1, uint16_t sensor_2);

private:
    HardwareSerialIMXRT* openMV; 
    uint8_t camera_value;

};
#endif