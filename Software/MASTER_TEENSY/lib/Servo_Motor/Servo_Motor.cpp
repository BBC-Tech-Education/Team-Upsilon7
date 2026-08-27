#include "Servo_Motor.h"

Servo_Motor::Servo_Motor(){}


/**
 * Attaches the servo motor to the given pin of the microcontroller.
 * 
 * @param pin The pin that the servo motor will be attachrd to
 * @param direction The direction that the servo motor will move. When direction = false 
 * the servo motor will move __ and when direction = true the servo will move _.
 * 
 * @return It will return 1 if the servo attaches and 0 if it's alread attached.
 */
int8_t Servo_Motor::attach(int8_t pin, bool direction){
    // Already attached -> do nothing
    if (servo_attached) {
        return 0;  
    }
    servo_direction = direction;
    
    myservo.attach(pin);

    servo_attached = true;
    return 1;
}



/**
 * Attaches the servo motor to the given pin of the microcontroller. It also set up the servo pulse
 * and turns it on. This function also let you chose the minimun and maximun values for:
 *
 * - Angle
 * 
 * - Pulse width
 * 
 * @param pin The pin that the servo motor will be attachrd to
 * @param direction The direction that the servo motor will move. When direction = false 
 * the servo motor will move __ and when direction = true the servo will move _.
 * @param min_angle The smallest angle the servo motor will move to.
 * @param max_angle The largest angle the servo motor will move to.
 * @param min_us The furthest __ postion the servo can move to.
 * @param max_us The furthest __ postion the servo can move to.
 * 
 * @return It will return 1 if the servo attaches and 0 if it's alread attached.
 */
int8_t Servo_Motor::attach(int8_t pin, bool direction, int8_t min_angle, int16_t max_angle, int16_t min_us, int16_t max_us){
    // Already attached -> do nothing
    if (servo_attached) {
        return 0;  
    }
    
    servo_direction = direction;
    myservo.attach(pin);

    minimum_angle = min_angle; maximum_angle = max_angle;
    minimum_us = min_us; maximum_us = max_us;

    servo_attached = true;
    return 1;
}



/**
 * Detaches the servo motor from the pin. Also turn the servo motor off by stoping the servo pulse.
 */
void Servo_Motor::detach(){
    myservo.detach(); 
    servo_attached = false;
}



/**
 * It set the speed of the servo as an int betwen -255 and 255
 * 
 * @param speed An int between -225 and 255
 */
void Servo_Motor::write_speed(int16_t speed){
    if (speed < -255) speed = -255;
    if (speed > 255) speed = 255;

    if (servo_direction == false){
    pulse_width = map(speed, -255, 255, minimum_us, maximum_us);
    } else{
      pulse_width = map(speed, -255, 255, maximum_us, minimum_us);  
    }
    myservo.writeMicroseconds(pulse_width);
}



/**
 * It set the speed of the servo as an float betwen -255.0 and 255.0
 * 
 * @param speed An float between -225 and 255
 */
void Servo_Motor::write_speed_float(float speed){
    int16_t speed_int = int(speed * 10);

    if (speed_int < -2550) speed_int = -2550;
    if (speed_int > 2550) speed_int = 2550;

    if (servo_direction == false){
        pulse_width = map(speed_int, -2550, 2550, minimum_us, maximum_us);
        myservo.writeMicroseconds(pulse_width);
    } else {
      pulse_width = map(speed_int, -2550, 2550, maximum_us, minimum_us); 
      myservo.writeMicroseconds(pulse_width); 
    }
    
}



/**
 * It set the angle of the servo as an integer betwen 0 and 180
 * 
 * @param angle The angle that the servo will move to
 * 
 */
void Servo_Motor::write_angle(int16_t angle){
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    if (servo_direction == false){
        pulse_width = map(angle, minimum_angle, maximum_angle, minimum_us, maximum_us);
        myservo.writeMicroseconds(pulse_width);
    
    } else{
      pulse_width = map(angle, minimum_angle, maximum_angle, maximum_us, minimum_us);  
      myservo.writeMicroseconds(pulse_width);
    }
}



/**
 * It set the angle of the servo by using the microseconds value
 * 
 * @param value The microseconds value that the servo will move to
 * 
 */
void Servo_Motor::write_microseconds(int16_t value){
    if (servo_direction == false){
        pulse_width = value; 
        myservo.writeMicroseconds(pulse_width);
    } else {
        pulse_width = map(value, maximum_us, minimum_us, minimum_us, maximum_us);
        myservo.writeMicroseconds(pulse_width);
    }
}



/**
 * Return the current speed of the servo motor
 * 
 * @return The speed as an int
 */
int16_t Servo_Motor::read_speed(){
    if(servo_direction == false){
        return (map(pulse_width, minimum_us, maximum_us, -255, 255));
    } else {
        return (map(pulse_width, minimum_us, maximum_us, 255, -255));
    }
}



/**
 * Return the current angle of the servo motor
 * 
 * @return The current servo angle
 */
int16_t Servo_Motor::read_angle(){
    if(servo_direction == false){
        return (map(pulse_width, minimum_us, maximum_us, minimum_angle, maximum_angle));
    } else {
        return (map(pulse_width, minimum_us, maximum_us, maximum_angle, minimum_angle));
    }
}



/**
 * Return the current microseconds of the servo motor
 * 
 * @return The current servo microseconds
 */
int16_t Servo_Motor::read_microseconds(){
    if(servo_direction == false){
        return pulse_width;
    } else {
        return map(pulse_width, minimum_us, maximum_us,maximum_us, minimum_us);
    }
}



/**
 * Says if the servo motor is attahced. True for attached and false for not
 * 
 * @return Attached value
 */
bool Servo_Motor::attached(){
    return servo_attached;
}

