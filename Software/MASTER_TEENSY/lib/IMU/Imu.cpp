#include "Imu.h"


void IMU::init()
{
    while(!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR.");
        delay(1000);
    }

    delay(500);
    bno.setExtCrystalUse(true);
    delay(1000);

    calibrate();
}



void IMU::read()
{   
    // reads the imu values
    bno.getEvent(&event);

    x = bearing_mod(event.orientation.x - initial_x);
    y = bearing_mod(-event.orientation.y - initial_y);
    z = bearing_mod(event.orientation.z - initial_z);

    #if DEBUG_BNO_DATA
    Serial.printf("X: %.2f\tY: %.2f\tZ: %.2f\n", x, y, z);
    #endif
}



float IMU::x_bearing_180()
{   
    // returns the x values
    return (x > 180.0f) ? (x - 360.0f) : x;
}

float IMU::y_bearing_180()
{
    // returns the y values
    return (y > 180.0f) ? (y - 360.0f) : y;
}

float IMU::z_bearing_180()
{
    // returns the z values
    return (z > 180.0f) ? (z - 360.0f) : z;
}



void IMU::set_target_bearing(int16_t bearing_change)
{
    // Changes the traget bearing 
    if (bearing_change < -180) {
        bearing_change += 360;
    } else if (bearing_change > 180) {
        bearing_change -= 180;
    }

    target_bearing += (360 + bearing_change);

    while (target_bearing > 360) {
        target_bearing -= 360;
    }
}



float IMU::x_angle_diff()
{
    // finds the different between the target beaing and current bearing
    float diff = x - (float)target_bearing;

    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if(diff < -180.0f) {
        diff += 360.0f;
    }

    return diff;
}



void IMU::calibrate()
{
    bno.getEvent(&event);

    initial_x = bearing_mod(event.orientation.x);
    initial_y = bearing_mod(-event.orientation.y);
    initial_z = bearing_mod(event.orientation.z);
}


float IMU::bearing_mod(float bearing)
{ // makes sure the bearing is with in range
    while (bearing < 0.0f) {
        bearing += 360.0f;
    }

    while (bearing > 360.0f) {
        bearing -= 360.0f;
    }

    return bearing;
}