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
        // Serial.println("HERE 6");
        while(openMV->available()) { // Finds the lastest value 
            camera_value = openMV->read();
        }
    }   
 
    #if DEBUG_CAMERA_DATA
    Serial.print("Camera Value: "); Serial.println(camera_value);
    #endif

    return camera_value; // returs the camera value

}

uint8_t CAMERA::process_data(uint16_t sensor_1, uint16_t sensor_2) {
    uint8_t cam_data = read_camera();
    if((sensor_1 > 255) || (sensor_2 > 255)) {
        past_data = 90;
        return 90;

    } else {
        float difference = (sensor_1 / sensor_2);
        if(difference < 1.0f) {
            difference = (1.0f / difference); 
        }
        if(LRF_DIFF < difference) {
            past_data = 90;
            return 90;
        }
    }
    
    // else if (sensor_1 < sensor_2) {
    //     if((sensor_1*LRF_DIFF) < sensor_2) {
    //         past_data = 90;
    //         return 90;
    //     }
    // } else {
    //     if((sensor_1*LRF_DIFF) < sensor_2) {
    //         past_data = 90;
    //         return 90;
    //     }   
    // }

    if((cam_data != 10) && (cam_data != 30)) {
        cam_data = 90;
    }

    if(past_data == cam_data) {
        return cam_data;
    }

    past_data = 90;
    return 90;
}

