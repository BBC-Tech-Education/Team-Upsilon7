#include "Distance_Sensors.h"

#define I2C_CLOCK 100000



void LRF_Array::init()
{
  // Starts wire
  Wire.begin();
  Wire.setClock(I2C_CLOCK);

  // Sets all distance sensors pins to low
  for (uint8_t i = 0; i < LRF_NUM; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(20);

  // assigns the xhut pin for short distance sensors
  for (uint8_t i = 0; i < LRF_SHORT_NUM; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(10);

    lrf_short[i].setTimeout(500);
    uint8_t attempts = 0;
    while (!lrf_short[i].init() && attempts < 10) {
      Serial.printf("Failed to detect and initialize sensor %d\n", i);
      attempts++;
      delay(100);
    }
    lrf_short[i].setAddress(0x2A + i);
    lrf_short[i].startContinuous();
  }

  // assigns the xhut pin for long distance sensors
  for (uint8_t i = 0; i < LRF_LONG_NUM; i++) {
    lrf_long[i].begin();

    lrf_long[i].InitSensor((0x2A + LRF_SHORT_NUM + i) * 2);
    lrf_long[i].VL53L4CX_StartMeasurement();
    lrf_long[i].VL53L4CX_ClearInterruptAndStartMeasurement();
  }
}



void LRF_Array::update()
{ 
  // reads short distance sensors
  for (uint8_t i = 0; i < LRF_SHORT_NUM; i++) {
    if (lrf_short[i].dataReady()) {
      int16_t val = lrf_short[i].read(false);

      if (val == 0 || val > LRF_SHORT_MAX_DIST) {
        values[i] = LRF_SHORT_MAX_DIST;
      } else if (val < LRF_SHORT_INVALID_DIST) {
        values[i] = 0;
      } else {
        values[i] = val;
      }
    }

    if (lrf_short[i].timeoutOccurred()) {
      values[i] = 0;
    }
  }

  // reads long distance sensors
  for (uint8_t i = 0; i < LRF_LONG_NUM; i++) {
    uint8_t newDataReady = 0;

    int status = lrf_long[i].VL53L4CX_GetMeasurementDataReady(&newDataReady);
    if (status != 0 || newDataReady == 0) {
      break;
    }

    VL53L4CX_MultiRangingData_t data;
    status = lrf_long[i].VL53L4CX_GetMultiRangingData(&data);

    if (status != 0) {
      values[LRF_SHORT_NUM + i] = 0;
      break;
    }

    int16_t val = 0;

    for (uint8_t j = 0; j < data.NumberOfObjectsFound; j++) {
      if (!data.RangeData[j].RangeStatus) {
        int16_t raw = data.RangeData[j].RangeMilliMeter;
        // if (raw < val) {
          val = raw;
        // }
        // Serial.print(i);
        // Serial.print("Distance Value "); Serial.print(j);Serial.print(": ");
        // Serial.println(val);
        break;
      }
    }

    // Serial.printf("LRF %d - raw: %d, previous: %d\t", i, val, values[LRF_SHORT_NUM + i]);

    // Serial.print(i); Serial.print("RAW LRF LONG: "); Serial.println(val);

    lrf_long[i].VL53L4CX_ClearInterruptAndStartMeasurement();
    
    if (val < LRF_LONG_INVALID_DIST || val > LRF_LONG_MAX_DIST) {
      val = 0;
    }
    values[LRF_SHORT_NUM + i] = (uint16_t)val;
  }
  // Serial.println();

  #if DEBUG_LRF_DATA
  Serial.printf("LRF RAW VALUES: ");
  for (uint8_t i = 0; i < LRF_NUM; i++) {
    Serial.printf("%d\t", values[i]);
  }
  Serial.println();
  #endif
}




uint16_t LRF_Array::get_value(uint8_t sensor)
{ 
  // returns a value of a desiered sensor
  return values[sensor];
}


uint16_t LRF_Array::get_side_value(uint8_t side)
{
  // reads one side of the robotics distance value
  uint16_t val1 = values[side * 2];
  uint16_t val2 = values[side * 2 + 1];

  if (val1 && val2) {
    return (val1 + val2) / 2;
  } else if (val1) {
    return val1;
  } else if (val2) {
    return val2;
  }
  return 0;
}



int16_t LRF_Array::tile_alignment()
{ // finds the difference between the two sides
  uint16_t max_left = max(values[2], values[3]);
  uint16_t max_right = max(values[4], values[5]);

  if (max_left > 200 && max_right > 200) {
    return 0;
  } else if (max_left > 200) {
    return get_side_value(LRF_RIGHT_SIDE) - ((TILE_WIDTH - ROBOT_WIDTH) / 2);
  } else if (max_right > 200) {
    return ((TILE_WIDTH - ROBOT_WIDTH) / 2) - get_side_value(LRF_LEFT_SIDE);
  }
  return get_side_value(LRF_RIGHT_SIDE) - get_side_value(LRF_LEFT_SIDE);
}