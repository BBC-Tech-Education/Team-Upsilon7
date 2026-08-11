#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <Arduino.h>
#include "Pins.h"
#include "VL53L4CD.h"
#include "vl53l4cx_class.h"
#include "Config.h"

class LRF_Array { 
  public:
    LRF_Array() {}

    void init();
    void update();

    uint16_t get_value(uint8_t sensor);
    uint16_t get_side_value(uint8_t side);
    int16_t tile_alignment();

  private:
    VL53L4CD lrf_short[LRF_SHORT_NUM];
    VL53L4CX lrf_long[LRF_LONG_NUM] = {VL53L4CX(&Wire, XSHUT8), VL53L4CX(&Wire, XSHUT9)};
    
    const uint8_t xshutPins[LRF_NUM] = {XSHUT0, XSHUT1, XSHUT2, XSHUT3, XSHUT4, XSHUT5, XSHUT6, XSHUT7, XSHUT8, XSHUT9};
    
    uint16_t values[LRF_NUM] = {0};
};



#endif


