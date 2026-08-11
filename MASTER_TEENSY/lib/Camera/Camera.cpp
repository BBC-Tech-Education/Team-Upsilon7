#include "Camera.h"


CAMERA::CAMERA(HardwareSerialIMXRT* serial) {
    openMV = serial; // sets up what serial the OpenMV camera uses
}


void CAMERA::init() {
    openMV->begin(115200); // starts serial
}



uint8_t CAMERA::read_camera() {
    camera_value = 0;
    //Serial.print("IS THERE STUFF"); Serial.println(openMV->available());
    // Serial.println("HERE 5");

    if (openMV->available()) { // Check if there is data
        //Serial.print("data");
        Serial.println("HERE 6");
        while(openMV->available()) { // Finds the lastest value 
            camera_value = openMV->read();
        }
    }   

    // if (openMV->available() >= 3) {
    //     //Serial.print("data");
    //     uint8_t data = 0;
    //     //Serial.println("HERE 6");
    //     while(openMV->available() >= 3) {
    //         //data = openMV->read();
    //         //Serial.print(data);
    //         data = openMV->read();
    //         //Serial.println("HERE 7");
    //         if(data == START_BYTE) {
    //             camera_value = openMV->read(); 
    //             data = openMV->read();
    //         } else {
    //             data = openMV->read(); 
    //         }
    //     }
    // }   
    #if DEBUG_CAMERA_DATA
    Serial.print("Camera Value: "); Serial.println(camera_value);
    #endif

    return camera_value; // returs the camera value

}