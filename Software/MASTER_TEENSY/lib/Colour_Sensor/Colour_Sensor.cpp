#include "Colour_Sensor.h"



void Colour_Sensor::init(){ 
    // Sets up the colour sesnor
    Wire2.begin();
    as7341.begin(AS7341_I2CADDR_DEFAULT, &Wire2);
    as7341.setATIME(5);
    as7341.setASTEP(50);
    as7341.setGain(AS7341_GAIN_64X);
    as7341.setLEDCurrent(255);
    as7341.enableLED(true);

    // Sets thresholds
    thresholds[BLACK] = 180;
    thresholds[BLUE] = 160;
    thresholds[SILVER] = 180;
    thresholds[RED] = 0;

    is_reading = false;
}



void Colour_Sensor::update(){
    if (!is_reading) { // starts reading the sesnor if not reading already
        as7341.startReading(); 
        is_reading = true;
        return; 
    }

    if (is_reading) {
        
        if (!as7341.checkReadingProgress()) { // if the sensor is reading the data
            return; 
        }
        // if the data is ready to go
        uint16_t readings[12];
        
        as7341.getAllChannels(readings);

        value.Channel_415nm   = readings[0];  // F1
        value.Channel_445nm   = readings[1];  // F2
        value.Channel_480nm   = readings[2];  // F3
        value.Channel_515nm   = readings[3];  // F4
        value.Channel_555nm   = readings[6];  // F5 
        value.Channel_590nm   = readings[7];  // F6
        value.Channel_630nm   = readings[8];  // F7
        value.Channel_680nm   = readings[9];  // F8
        value.Channel_Clear    = readings[10]; // Clear
        value.Channel_Near_IR  = readings[11]; // NIR

        #if DEBUG_COLOUR_DATA
        Serial.printf("415nm: %d\t", value.Channel_415nm);
        Serial.printf("445nm: %d\t", value.Channel_445nm);
        Serial.printf("480nm: %d\t", value.Channel_480nm);
        Serial.printf("515nm: %d\t", value.Channel_515nm);
        Serial.printf("555nm: %d\t", value.Channel_555nm);
        Serial.printf("590nm: %d\t", value.Channel_590nm);
        Serial.printf("630nm: %d\t", value.Channel_630nm);
        Serial.printf("680nm: %d\t", value.Channel_680nm);
        Serial.printf("Clear: %d\t", value.Channel_Clear);
        Serial.printf("IR: %d\n", value.Channel_Near_IR);
        #endif
        
        as7341.startReading(); 
        
        is_reading = false; 
    }
}



uint8_t Colour_Sensor::see_colour(uint8_t colour) {
    if (colour == BLACK) { // Check if the colour is black
        return ((thresholds[BLACK] > value.Channel_Clear) && (value.Channel_Clear != 0));
        
    } else if (colour == SILVER) { // Check if the colour is silver
        return ((thresholds[SILVER] > value.Channel_445nm) && (value.Channel_445nm != 0));

    } else if (colour == BLUE){ // Check if the colour is blue
        return ((thresholds[BLUE] > value.Channel_480nm) && (value.Channel_480nm != 0));

    } else if (colour == RED) { // Check if the colour is red
       
        // TODO

    }

    return 0;
}



void Colour_Sensor::calibrate(uint8_t colour) { // not used

    // TODO: incorporate threshold tolerance

    if (colour == BLACK) {
        thresholds[BLACK] = value.Channel_Clear;
    } else if (colour == BLUE){
        thresholds[BLUE] = value.Channel_480nm;
    } else if (colour == SILVER){
        thresholds[SILVER] = value.Channel_Clear;
    } else if (colour == RED){
        
        // TODO

    }
}