#include "Motor_Array.h"


Motor_Array::Motor_Array(){};



void Motor_Array::init(uint8_t pin_lf, uint8_t pin_lb, uint8_t pin_rf, uint8_t pin_rb){
    motor_lf.attach(pin_lf, false);
    motor_lb.attach(pin_lb, false);
    motor_rf.attach(pin_rf, true);
    motor_rb.attach(pin_rb, true);

}



void Motor_Array::move(int16_t speed_left, int16_t speed_right){
    // Moves all motors based upon an int input
    Speed_Value_Left = speed_left;
    Speed_Value_Right = speed_right;

    motor_lf.write_speed(speed_left);
    motor_lb.write_speed(speed_left);
    motor_rf.write_speed(speed_right);
    motor_rb.write_speed(speed_right);

}



void Motor_Array::move_float(float speed_left, float speed_right){
     // Move all motors based upon an float input
    Speed_Value_Left = uint16_t(speed_left);
    Speed_Value_Right = uint16_t(speed_right);
    
    motor_lf.write_speed_float(speed_left);
    motor_lb.write_speed_float(speed_left);
    motor_rf.write_speed_float(speed_right);
    motor_rb.write_speed_float(speed_right);

}



int16_t Motor_Array::current_speed(){
    return Speed_Value_Left, Speed_Value_Right;
}