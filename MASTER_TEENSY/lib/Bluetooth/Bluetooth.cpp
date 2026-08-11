#include "Bluetooth.h"


Bluetooth_Module::Bluetooth_Module(uint8_t RX_Pin, uint8_t TX_Pin)
 :BT_Serial(RX_Pin, TX_Pin)
{}



void Bluetooth_Module::init(){
    BT_Serial.begin(9600);
}



u_int16_t Bluetooth_Module::read(){
    if (BT_Serial.available()) {
        return BT_Serial.read();
    } else {
        return 0;
    }
}



void Bluetooth_Module::write(u_int16_t value){
    BT_Serial.write(value);
}