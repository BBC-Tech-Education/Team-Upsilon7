// V2
#include <math.h>
#include <Arduino.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <errno.h>
#include <algorithm>
#include <iterator> 
#include <iostream>

#include <Config.h>
#include <Bluetooth.h>
#include <Colour_Sensor.h>
#include <Distance_Sensors.h>
#include <Imu.h>
#include <Switch.h>
#include <Motor_Array.h>
#include <Servo_Motor.h>
#include <PID.h>
#include <Mapping.h>
#include <Camera.h>
#include <Pins.h>
#include <Adafruit_NeoPixel.h>






enum state_values {
  INIT, FORWARD, LEFT, RIGHT, BACK, MAPPING, BLACK_TILE, BLUE_TILE, VICTIM, VICTIM_DROPPER,
  RAMP, RAMP_FORWARD, RESET_SWITCH
};

//=========//=======================//=======================//=======================
uint8_t robot_init = true;
uint8_t state_data = 0;

uint8_t next_state = 0;


enum state_values current_state = INIT;
std::vector<int8_t> past_states = {100}; // should be 100

// ---------------- Distances ----------------
uint16_t target_distance;
uint16_t target_distance_back;
uint16_t average_distance_front;
uint16_t average_distance_back;
uint8_t front_target_dis_bad = false;


// // ---------------- Black Tile ----------------
uint8_t black_state_data[3] = {0, 0, 0};
uint8_t turn_backward = 0;


// // ---------------- Ramps ----------------
int16_t ramp_info[4] = {0, 0, 0, 0};
std::vector<float> average_values;
uint16_t num_of_values = 0;
float ramp_angle = 0;


// ---------------- Cameras ----------------
uint8_t camera_data[2] = {0, 0};
uint8_t past_victims = 0;


// // ---------------- Timers ----------------
// uint32_t timer_start;
uint32_t timer_led;
uint32_t timer_led_flash;
uint32_t timer_blue;


// // ---------------- Other ----------------
bool led_state = false;

uint8_t limit_switches; // infomation limit swtiches
uint8_t past_limit = 0; // after the limit swtiches have been actvated
uint8_t limit_move = 0; // when the limit switches are double actavited

uint8_t reset_switch_data = 0;
float stuck_speed = 0;



// --------------------------------- Sensors ---------------------------------
Adafruit_NeoPixel neopixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
IMU bno;
Motor_Array motors;
LRF_Array lrfs;
Colour_Sensor colour_sensor;

CAMERA camera_left = CAMERA(&Serial1);
CAMERA camera_right = CAMERA(&Serial2);

// Switch ls_fl();
// Switch ls_fr();
// Switch ls_bl();
// Switch ls_rl();

Switch reset_switch;

Servo_Motor dropper;
Servo_Motor front_pivot;
Servo_Motor back_pivot;

Mapping maze_map;
PID motor_PID = PID(7.0f,0.0f,1.0f); //0.6, 0, 0.3 //17



//=======================//=======================//=======================//=======================

// ---------------------------------- Movement ----------------------------------
void movement_forward(int speed) {
  motors.move(speed, speed);
}


void movement_forward_corrected(float speed, float offset) {
  uint32_t current_time = millis();

  float angle_off = atan2f(lrfs.tile_alignment(), PID_DIS);
  float angle_off_deg = 180/PI * angle_off;

  float angle_diff = bno.x_angle_diff() - angle_off_deg;

  float pid_value = motor_PID.update(angle_diff, 0);
  // timer_PID = millis();

  // Serial.print("PID Value"); Serial.println(pid_value); 

  motors.move_float((speed + pid_value - offset) , (speed - pid_value + offset));
}


void movement_turning(int speed) {
  motors.move(speed, -speed);  
}



// ---------------------------------- Lrf ----------------------------------
uint16_t front_lrf()
{
  uint16_t short_dist = lrfs.get_side_value(LRF_FRONT_SHORT_SIDE);
  uint16_t long_dist = lrfs.get_side_value(LRF_FRONT_LONG_SIDE);
  if (short_dist && long_dist) {
    if (short_dist > (LRF_SHORT_MAX_DIST - 50)) {
      return long_dist;
    } else {
      return short_dist;
    }
  } else if (long_dist) {
    return long_dist;
  } else if (short_dist) {
    return short_dist;
  }
  return 0;
}


uint8_t distance_conditions(uint8_t direction) {
  uint16_t distance = 0;
  uint16_t targ_distance = 0;

  uint16_t short_dist = lrfs.get_side_value(LRF_FRONT_SHORT_SIDE);
  uint16_t long_dist = lrfs.get_side_value(LRF_FRONT_LONG_SIDE);
  uint16_t back_dist = lrfs.get_side_value(LRF_BACK_SIDE);

  if (short_dist == 0 || short_dist >= LRF_SHORT_MAX_DIST) {
    if (back_dist == 0 || back_dist >= LRF_SHORT_MAX_DIST) {
      if (long_dist == 0 || long_dist >= LRF_LONG_MAX_DIST || front_target_dis_bad) {
        lrfs.update();
        Serial.println("WELLLLLLLLLL THIS SUCKS");
        delay(50);
        
        return distance_conditions(direction);
      }
      distance = long_dist;
      targ_distance = target_distance;
    } else {
      distance = back_dist;
      targ_distance = target_distance_back;
      direction = !direction;
    }
  } else {
    if (front_target_dis_bad) {
      if (back_dist == 0 || back_dist >= LRF_SHORT_MAX_DIST) {
        lrfs.update();
        Serial.println("WELLLLLLLLLL THIS SUCKS");
        delay(50);
        if(state_data & BMSK_S_RESETING) {
          current_state = RESET_SWITCH;
          state_data |= BMSK_S_NEW_STATE;
        }
        return distance_conditions(direction);
      } else {
        distance = back_dist;
        targ_distance = target_distance_back;
        direction = !direction;
      }
    } else {
      distance = short_dist;
      targ_distance = target_distance;
    }
  }

  return (distance < targ_distance) ? direction : !direction;
}



// ---------------------------------- Mapping ----------------------------------
void set_tile_info(bool victim, bool explored, bool black, bool silver, 
  bool blue, bool red, bool obstacle, bool ramp) // remove explored
{ 
  uint8_t tile_info = 0;
  bool info_list[8] = {victim, explored, black, silver, blue, red, obstacle, ramp};
  uint8_t bmsk_list[8] = {BMSK_T_VICTIM, BMSK_T_EXPLORED, BMSK_T_BLACK, BMSK_T_SILVER, 
    BMSK_T_BLUE, BMSK_T_RED, BMSK_T_OBSTACLE, BMSK_T_RAMP};
  
  for (int8_t i = 0; i < 8; i++)
  {
    if (info_list[i]) {
      tile_info |= bmsk_list[i];
    }
  }
  maze_map.set_current_info(tile_info);
}


// ---------------------------------- LED ----------------------------------
void led_on(){
  //digitalWrite(LED_PIN, HIGH);
  neopixel.setPixelColor(0, neopixel.Color(0, 0, 150));
  neopixel.show();
}


void led_off(){
  // digitalWrite(LED_PIN, LOW);
  neopixel.setPixelColor(0, neopixel.Color(0, 0, 0));
  neopixel.show();
}


void led_flashing(uint16_t speed) {
  uint32_t time_between_flash = millis() - timer_led_flash;

  if (time_between_flash > speed) {
    if(led_state) {
      led_off();
      led_state = false;
      timer_led_flash = millis();
    } else {
      led_on();
      led_state = true;
      timer_led_flash = millis();
    }
  }
}



// ---------------------------------- Dropper ----------------------------------
void drop_package(int8_t side) {
  if (side == -1) {
    dropper.write_angle(0);
    delay(1000);
    dropper.write_angle(130);
    delay(1000);
    dropper.write_angle(90);
    delay(1000);
  } else if (side == 1) {
    dropper.write_angle(180);
    delay(1000);
    dropper.write_angle(50);
    delay(1000);
    dropper.write_angle(90);
    delay(1000);
  } else {
    Serial.println("Enter in a Valid Side");
  }
}



// ---------------------------------- Victim ----------------------------------
void victim_led(){
  led_flashing(500);
}


void victim_package(int8_t side, uint8_t number){
  led_flashing(500);

  if(number == 1) {
    drop_package(side);
  }
  if(number == 2) {
    drop_package(side);
    drop_package(side);
  }
}



// ---------------------------------- Lrf servo ----------------------------------
void pivoting_lrf_front(){
  front_pivot.write_angle((90-int16_t(bno.y_bearing_180())));
}


void pivoting_lrf_back(){
  back_pivot.write_angle((90+int16_t(bno.y_bearing_180())));
}



// ---------------------------------- Imu Average ----------------------------------
std::vector<float> standard_deviation() {
    // variable to store sum of the given data
    float sum = 0;
    for (int i = 0; i < num_of_values; i++) {
        sum += average_values[i];
    }

    // calculating mean
    float mean = sum / num_of_values;

    // temporary variable to store the summation of square
    // of difference between individual data items and mean
    float values = 0;

    for (int i = 0; i < num_of_values; i++) {
        values += pow(average_values[i] - mean, 2);
    }

    // variance is the square of standard deviation
    float variance = values / num_of_values;

    // calculating standard deviation by finding square root
    // of variance
    float standardDeviation = sqrt(variance);

    // printing standard deviation
    //printf("%.2f\n", standardDeviation);

    return {mean, standardDeviation};
}


void imu_average_s() {
  average_values.clear();
  num_of_values = 0;
}


std::vector<float> imu_average_u(){
  num_of_values += 1;
  average_values.push_back(bno.y_bearing_180());

  // find std and return that
  return standard_deviation();
}


float imu_average_e(){
  std::vector<float> std_dev = standard_deviation();
  float lower_bound = std_dev[0] - std_dev[1];
  float upper_bound = std_dev[0] + std_dev[1];

  uint32_t values_used = 0;
  float agg_value = 0.0f;

  for (uint32_t i = 0; i < num_of_values; i++)
  {
    if((average_values[i] >= lower_bound) && (average_values[i] <= upper_bound)) {
      agg_value += average_values[i];
      values_used ++;
    }
  }
  Serial.print("Values: "); Serial.print("\t"); Serial.print(agg_value); Serial.print("\t"); 
  Serial.println(values_used);

  ramp_angle = (agg_value/values_used);
  Serial.print("Ramp Angle: "); Serial.println(ramp_angle);
  return ramp_angle;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ---------------------------------- States -----------------------------------
void state_init() {
  // ----------------- Setup -----------------
  

  // ------------ Exit Conditions ------------
  if(robot_init) {
    current_state = MAPPING;
    state_data |= BMSK_S_NEW_STATE;
    return;
  }
  
  // ---------------- Function ----------------
  
}



void state_forward() {
  // ----------------- Setup -----------------
  if (state_data & BMSK_S_NEW_STATE) {
    state_data &= ~BMSK_S_SWITCH_TILE;
    past_states.push_back(FORWARD);
    state_data &= ~BMSK_S_NEW_STATE;

    // +++++ How will this change with different distances at front and back)
    // where do i need to move to
    uint16_t front_dist = front_lrf();
    if (front_dist == LRF_SHORT_MAX_DIST) {
      front_target_dis_bad = true;
    } else {
      front_target_dis_bad = false;
      int16_t distance_to_move = ((front_dist % 300) + (300 - ROBOT_LENGTH)/2 + ROBOT_LENGTH); // find how much to move forward
      int16_t target_dis = front_lrf() - distance_to_move + MOVEMENT_OFFSET;// 150;
      target_distance = max(target_dis, 60);
    }

    
    #if DEBUG_TARGET_DISTANCES 
      //Serial.print("Target Distance:"); Serial.println(target_dis);
    #endif


    uint16_t back_dist = lrfs.get_side_value(LRF_BACK_SIDE);
    if (back_dist == LRF_SHORT_MAX_DIST) {
      target_distance_back = 0;
    } else {
      target_distance_back = back_dist + (300 - (back_dist % 300) + (300 - ROBOT_LENGTH) / 2);
    }

    Serial.printf("Target Distance Front: %d\tTarget Distance Back: %d\n", target_distance, target_distance_back);

    average_distance_front = ((target_distance + front_lrf())/2);
    average_distance_back = (target_distance_back + back_dist) / 2;
  }

  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;

  } else if((state_data & BMSK_S_VICTIM_FOUND) && !past_victims) {
    current_state = VICTIM;
    state_data |= BMSK_S_NEW_STATE;

  } else if (colour_sensor.see_colour(BLACK)) {
    current_state = BLACK_TILE;
    state_data |= BMSK_S_NEW_STATE;

  } else if(distance_conditions(1)) {
    current_state = MAPPING;
    state_data |= BMSK_S_NEW_STATE;
    
  } else if ((bno.y_bearing_180() > RAMP_TOL_UP) || (bno.y_bearing_180() < -RAMP_TOL_DOWN)) { //detected a ramp
    //if debris is messing this up, up the ramp tolerance
    current_state = RAMP;
    state_data |= BMSK_S_NEW_STATE;
  }

  if(state_data & BMSK_S_NEW_STATE) {
    return;
  }

  // ---------------- Function ----------------
  movement_forward_corrected(FORWARD_SPEED, 0);

  if ((!(state_data & BMSK_S_SWITCH_TILE) && front_lrf() <= average_distance_front)
   || (!(state_data & BMSK_S_SWITCH_TILE) && lrfs.get_side_value(LRF_BACK_SIDE) <= average_distance_back)) { //haven't switched tile yet, lrf is telling I've moved
    state_data |= BMSK_S_SWITCH_TILE;
    maze_map.switch_current_tile();
    past_victims = maze_map.victims(); //check if victims have been saved
  }
}



void state_left() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);

    bno.set_target_bearing(-90);
    past_states.push_back(LEFT);
    state_data &= ~BMSK_S_NEW_STATE;
  }

  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;

  } else if((state_data & BMSK_S_VICTIM_FOUND) && !past_victims) {
    current_state = VICTIM;
    state_data |= BMSK_S_NEW_STATE;

  } else if(bno.x_angle_diff() < ANGLE_TOL) {
    current_state = FORWARD;
    state_data |= BMSK_S_NEW_STATE;
  } 
  
  // ---------------- Function ----------------
  movement_turning(-TURN_SPEED); 
}



void state_right() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);

    bno.set_target_bearing(90);
    past_states.push_back(RIGHT);
    state_data &= ~BMSK_S_NEW_STATE;
  }

  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;

  } else if((state_data & BMSK_S_VICTIM_FOUND) && !past_victims) {
    current_state = VICTIM;
    state_data |= BMSK_S_NEW_STATE;

  } else if(bno.x_angle_diff() > -ANGLE_TOL) {
    current_state = FORWARD;
    state_data |= BMSK_S_NEW_STATE;
  } 
  
  // ---------------- Function ----------------
  movement_turning(TURN_SPEED); 
}



void state_back() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);

    bno.set_target_bearing(90);
    past_states.push_back(RIGHT);
    state_data &= ~BMSK_S_NEW_STATE;
  }

  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;

  } else if((state_data & BMSK_S_VICTIM_FOUND) && !past_victims) {
    current_state = VICTIM;
    state_data |= BMSK_S_NEW_STATE;

  } else if(bno.x_angle_diff() > -ANGLE_TOL) {
    if(turn_backward == 1) {
      turn_backward = 0;
      current_state = FORWARD;
      state_data |= BMSK_S_NEW_STATE;
    } else {
      turn_backward += 1;
      state_data |= BMSK_S_NEW_STATE;
    }
    
  } 
  // ---------------- Function ----------------
  movement_turning(TURN_SPEED);
  
}



void state_mapping() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) { 
    movement_forward(0);

    past_states.push_back(MAPPING);
    state_data &= ~BMSK_S_NEW_STATE;

    delay(200);
    lrfs.update();
    colour_sensor.update();

    if(colour_sensor.see_colour(SILVER)) {
      set_tile_info(false,false,false,true,false,false,false,false);
    }

    // set up target distances
  }

  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;

  } else if (colour_sensor.see_colour(BLUE) && !(state_data & BMSK_S_BLUE_TILE_DONE)) { //sees blue for the first time
    set_tile_info(false,false,false,false,true,false,false,false);
    current_state = BLUE_TILE;
    state_data |= BMSK_S_NEW_STATE;

  } else if (state_data & BMSK_S_MAPPING_COMPLETED) { //mapping function completed
    state_data &= ~BMSK_S_BLUE_TILE_DONE;

    if (next_state == 1) { //mapping says to move left
      current_state = LEFT;
      state_data |= BMSK_S_NEW_STATE;

    } else if (next_state == 2) { //mapping says to move forward
      //*p_timer_start = millis();
      current_state = FORWARD;
      state_data |= BMSK_S_NEW_STATE;

    } else if (next_state == 3) {  //mapping says to move right
      current_state = RIGHT;
      state_data |= BMSK_S_NEW_STATE;

    } else { //mapping says to move back
      current_state = BACK;
      state_data |= BMSK_S_NEW_STATE;
    }
    state_data &= ~BMSK_S_MAPPING_COMPLETED;
  }
  if(state_data & BMSK_S_NEW_STATE){
    return;
  }

  // ---------------- Function ----------------
  uint8_t tile_available[4]= {0};
  uint16_t side_distances[4] = {
      lrfs.get_side_value(LRF_LEFT_SIDE), 
      lrfs.get_side_value(LRF_FRONT_SHORT_SIDE),
      lrfs.get_side_value(LRF_RIGHT_SIDE), 
      lrfs.get_side_value(LRF_BACK_SIDE)
  };


  for (uint8_t i = 0; i < 4; i++) {
      if(side_distances[i] > 250) { // may need to be changed
        tile_available[i] = 1;
      }
  }
  
  maze_map.create_tiles(bno.x_bearing_180(), tile_available);
  next_state = maze_map.mapping_alg(bno.x_bearing_180());
  
  state_data |= BMSK_S_MAPPING_COMPLETED;
}



void state_black_tile() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);

    past_states.push_back(MAPPING);
    state_data &= ~BMSK_S_NEW_STATE;

    set_tile_info(false, true, true, false, false, false, false, false);

    uint16_t back_dist = lrfs.get_side_value(LRF_BACK_SIDE);
    int16_t distance_to_move;
    if (back_dist == LRF_SHORT_MAX_DIST) {
      target_distance_back = 0;
    } else {
      distance_to_move = ((back_dist % 300) - ((300 - ROBOT_LENGTH)/2));
      target_distance_back = back_dist - distance_to_move;
    }
    
    uint16_t front_dist = front_lrf();
    if (front_dist == LRF_SHORT_MAX_DIST) {
      front_target_dis_bad = true;
    } else {
      front_target_dis_bad = false;
      distance_to_move = 300 - (front_dist % 300) + (300 - ROBOT_LENGTH)/2;
      target_distance = front_dist + distance_to_move;
    }
  }
  

  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;

  } else if (distance_conditions(0)) {
    current_state = MAPPING;
    state_data |= BMSK_S_NEW_STATE;
  } 
  
  // ---------------- Function ----------------
  movement_forward(-50);
}



void state_blue_tile() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) { //first time in this state
    //movement_forward(0);
    timer_blue = millis() ;
    state_data &= ~BMSK_S_NEW_STATE;
    past_states.push_back(BLUE_TILE);
  }

  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;
  } else if ((millis()- timer_blue) > 5000) {
    state_data |= BMSK_S_BLUE_TILE_DONE;
    current_state = MAPPING;
    state_data |= BMSK_S_NEW_STATE;

  } else {
    // movement_forward(0);
  }
  
  // ---------------- Function ----------------
}



void state_victim() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);
    state_data &= ~BMSK_S_NEW_STATE;
    past_states.push_back(VICTIM);
    timer_led = 0;

  }
  
  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;
  } else if((timer_led - millis()) > 5000) {
    set_tile_info(true, false, false, false, false, false, false, false);
    past_victims = 1;

    if (camera_data[CAM_LEFT] == 30|| camera_data[CAM_RIGHT] == 30) {
      if(past_states[past_states.size() - 2] == LEFT){
        current_state = LEFT;

      } else if (past_states[past_states.size() - 2] == RIGHT) {
        current_state = RIGHT;

      } else if (past_states[past_states.size() - 2] == BACK) {
        current_state = BACK;

      } else if(past_states[past_states.size() - 2] == FORWARD) {
        current_state = FORWARD;

      } else if(past_states[past_states.size() - 2] == RAMP_FORWARD) {
        current_state = RAMP_FORWARD;
      }
    } else {
      current_state = VICTIM_DROPPER;
      state_data |= BMSK_S_NEW_STATE;
    }
  } else{
    led_flashing(500);
  }
  if(state_data & BMSK_S_NEW_STATE){
    return;
  }


  // ---------------- Function ----------------
  movement_forward(0);
  if(camera_data[CAM_LEFT] == 70 || camera_data[CAM_RIGHT] == 70) {
    // movement_forward(0);
  } else {
    if(timer_led == 0) {
      timer_led = millis();
    }
  }
}



void state_victim_dropper() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);
    state_data &= ~BMSK_S_NEW_STATE;
    past_states.push_back(VICTIM_DROPPER);

  }
  
  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;
    
  } else if(state_data & BMSK_S_VICTIM_COMPLETED) {
    if(past_states[past_states.size() - 3] == 102){
      current_state = LEFT;

    } else if (past_states[past_states.size() - 3] == 103) {
      current_state = RIGHT;

    } else if (past_states[past_states.size() - 3] == 104) {
      current_state = BACK;

    } else if(past_states[past_states.size() - 3] == 101) {
      current_state = FORWARD;
    } else if(past_states[past_states.size() - 2] == RAMP_FORWARD) {
        current_state = RAMP_FORWARD;
    }
    state_data &= ~BMSK_S_VICTIM_COMPLETED;
  } 
  if (state_data & BMSK_S_NEW_STATE) {
    return;
  }


  // ---------------- Function ----------------
  movement_forward(0);
  if(camera_data[CAM_LEFT] == 10 || camera_data[CAM_LEFT] == 20) {
    victim_package(-1, (camera_data[CAM_LEFT])/10);
     state_data |= BMSK_S_VICTIM_COMPLETED;
  }

  if(camera_data[CAM_RIGHT] == 10 || camera_data[CAM_RIGHT] == 20) {
    victim_package(1, (camera_data[CAM_RIGHT])/10);
     state_data |= BMSK_S_VICTIM_COMPLETED;
  }
}



void state_ramp() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);
    // reset variables
    state_data &= ~BMSK_S_NEW_STATE;

    past_states.push_back(RAMP);

    Serial.println("DO I GET HERE 1");

    // what types of ramp is this
    if (bno.y_bearing_180() > RAMP_TOL_UP) {
      ramp_info[RAMP_DIR] = 1; // up
    } else {
      ramp_info[RAMP_DIR] = 2; // down
    }

    ramp_info[RAMP_DIS] = lrfs.get_side_value(LRF_FRONT_SHORT_SIDE);
    
    imu_average_s();

    Serial.println("DO I GET HERE 2");
  }
  
  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;
    
  } else if((bno.y_bearing_180() > -4) && (bno.y_bearing_180() < 4)) { //moving forward off the ramp
    current_state = RAMP_FORWARD;
    state_data |= BMSK_S_NEW_STATE;
  } 
  if (state_data & BMSK_S_NEW_STATE) {
    return;
  }


  // ---------------- Function ----------------
  if(ramp_info[RAMP_DIR] == 1) { //robot going up
    //Serial.println("DO I GET HERE 5");

    //moving forward with a bit more power
    movement_forward_corrected(float(FORWARD_SPEED*1.3f), 0);


    //IMU average to give you a range of the ramp angle, if in range get LRF value
    std::vector<float> bno_data = imu_average_u();
    //Serial.println(num_of_values);
    float range[2] = {(fabs(bno_data[0]*0.95f)), 
                      (fabs(bno_data[0])*1.05f)};

    if (((range[0] <= fabs(bno.y_bearing_180())) || (fabs(bno.y_bearing_180()) <= range[1]))) {
      ramp_info[RAMP_DIS] = lrfs.get_side_value(LRF_BACK_SIDE);
    }

  } else { //robot going down
    // Serial.println("DO I GET HERE 6");
    movement_forward_corrected(float(FORWARD_SPEED*0.8f), 0);
    float change_amount = fabs(bno.y_bearing_180()) /abs(ramp_info[RAMP_CUR_ANG]);
    ramp_info[RAMP_CUR_ANG] = bno.y_bearing_180();
    if((change_amount > 1.03f) && (ramp_info[RAMP_DIS_READ] < 10)) { //got a good distance of the ramp
      ramp_info[RAMP_DIS] = lrfs.get_side_value(LRF_FRONT_LONG_SIDE);    
    }
    else { //you are at the bottom
      if (ramp_info[RAMP_DIS_READ] < 10) {
        ramp_info[RAMP_DIS_READ] ++;
      }
    }
  }
}



void state_ramp_forward() {
  // ----------------- Setup -----------------
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);

    state_data &= ~BMSK_S_NEW_STATE;
    past_states.push_back(RAMP_FORWARD);
    imu_average_e(); //ending the average of ramp angle

    // find how much to move
    // int16_t distance_to_move = ((lrfs.get_side_value(LRF_FRONT_SHORT_SIDE) % 300) + 
    // (300 - ROBOT_LENGTH)/2 + ROBOT_LENGTH); // find how much to move forward

    int16_t distance_to_move = 
    ((300 - ROBOT_LENGTH)/2 + ROBOT_LENGTH)-
    (290-(lrfs.get_side_value(LRF_FRONT_SHORT_SIDE) % 300));
    int16_t target_dis = front_lrf() - distance_to_move;
    target_distance = target_dis < 60 ? 60 : target_dis;
    front_target_dis_bad = false;


    uint16_t back_dist = lrfs.get_side_value(LRF_BACK_SIDE);
    if (back_dist == LRF_SHORT_MAX_DIST) {
      target_distance_back = 0;
    } else {
      target_distance_back = lrfs.get_side_value(LRF_BACK_SIDE) + distance_to_move;
    }

    // Serial.printf("Target Distance Front: %d\tTarget Distance Back: %d\n", target_distance, target_distance_back);


    // Distance info
    int16_t excess_distance = 0;
    int16_t ramp_length = 0;
    
    excess_distance = fabs(ROBOT_HEIGHT / tanf(ramp_angle* (PI/180)));
    ramp_length = (ROBOT_HEIGHT+ ramp_info[RAMP_DIS]) - excess_distance;
    
    
    int16_t hoz_length = ramp_length * cosf(ramp_angle* (PI/180));
    uint8_t num_of_tiles = uint8_t(round(fabs(hoz_length/300)));
    int16_t ramp_height = ramp_length * sinf(ramp_angle* (PI/180));

    uint8_t num_of_height = uint8_t(round(fabs(ramp_height/150)));

    maze_map.ramp(bno.x_bearing_180(), ramp_info[RAMP_DIR], num_of_tiles, num_of_height);

    #if DEBUG_RAMP_LENGTH
    Serial.print("Ramp crap: ");

    Serial.print(ramp_info[RAMP_DIS]); Serial.print("\t"); 
    Serial.print(excess_distance); Serial.print("\t");
    Serial.print(ramp_length); Serial.print("\t");
    Serial.print(hoz_length); Serial.print("\t");
    Serial.print(num_of_tiles); Serial.print("\t");
    Serial.print(ramp_angle); Serial.print("\t");
    Serial.print(ramp_height); Serial.print("\t");
    Serial.print(num_of_height); Serial.println("\t");
    #endif
  }
  
  // ------------ Exit Conditions ------------
  if(state_data & BMSK_S_RESETING) {
    current_state = RESET_SWITCH;
    state_data |= BMSK_S_NEW_STATE;
    
  } else if (state_data & BMSK_S_VICTIM_FOUND) {
    current_state = VICTIM;
    state_data |= BMSK_S_NEW_STATE;

  } else if (colour_sensor.see_colour(BLACK)) {
    current_state = BLACK_TILE;
    state_data |= BMSK_S_NEW_STATE;

  } else if ((distance_conditions(1)) && (fabs(bno.y_bearing_180()) < 4)) { //off ramp and centre of next tile
    current_state = MAPPING;
    state_data |= BMSK_S_NEW_STATE;
  } 
  if (state_data & BMSK_S_NEW_STATE) {
    return;
  }


  // ---------------- Function ----------------
  movement_forward_corrected((FORWARD_SPEED-10), 0);
}



void state_reset_switch() {
  if(state_data & BMSK_S_NEW_STATE) {
    movement_forward(0);
    past_states.push_back(RESET_SWITCH);
    state_data &= ~BMSK_S_NEW_STATE;

    maze_map.soft_reset();
    state_data &= ~BMSK_S_MAP_RESETED;
    state_data |= BMSK_S_RESETING;

  }
  
  // ------------ Exit Conditions ------------  
  if (state_data & BMSK_S_MAP_RESETED) {
    current_state = MAPPING;
    state_data |= BMSK_S_NEW_STATE;
    state_data &= ~BMSK_S_RESETING;

    delay(100);
    bno.read();
    lrfs.update();
    return;
  }


  // ---------------- Function ----------------
  movement_forward(0);
  if(!reset_switch_data) {
    state_data |= BMSK_S_MAP_RESETED;
  }
  
}



void state_update() {
  switch (current_state) {
    case INIT:
      state_init();
      break;

    case FORWARD:
      state_forward();
      break;

    case LEFT:
      state_left();
      break;

    case RIGHT:
      state_right();
      break;

    case BACK:
      state_back();
      break;

    case MAPPING:
      state_mapping();
      break;

    case BLACK_TILE:
      state_black_tile();
      break;

    case BLUE_TILE:
      state_blue_tile();
      break;

    case VICTIM:
      state_victim();
      break;

    case VICTIM_DROPPER:
      state_victim_dropper();
      break;

    case RAMP:
      state_ramp();
      break;

    case RAMP_FORWARD:
      state_ramp_forward();
      break;

    case RESET_SWITCH:
      state_reset_switch();
      break;

    default:
      break;
  }
}





void setup() {
  neopixel.begin();

  lrfs.init();
  motors.init(MOTOR_LF,MOTOR_LB,MOTOR_RF,MOTOR_RB);

  bno.init();
  bno.set_target_bearing(0);

  // ls_1.init(7);

  colour_sensor.init();

  camera_left.init();
  camera_right.init();

  back_pivot.attach(PIVET_BACK_PIN, false);
  front_pivot.attach(PIVET_FRONT_PIN, false);
  dropper.attach(DROPPER_PIN, true, 0, 180, 900, 2000);

  reset_switch.init(RESET_PIN);

  maze_map.init();

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
}



void loop() {
  reset_switch_data = reset_switch.read();
  if(reset_switch_data){
    state_data |= BMSK_S_RESETING;
  } else {
    state_data &= ~BMSK_S_RESETING;
  }
  #if DEBUG_STATE_DATA
  Serial.print("Current State: "); Serial.print(current_state); 
  Serial.print(" Reset Switch Data: "); Serial.print(reset_switch_data);
  Serial.print(" Target Distance: "); Serial.println(target_distance);
  #endif

  lrfs.update();

  // victims, comment if not needed
  camera_data[CAM_LEFT] = camera_left.read_camera();
  camera_data[CAM_RIGHT] = camera_right.read_camera();
  if((camera_data[CAM_LEFT] && !past_victims && camera_data[CAM_LEFT] != 90) ||
    (camera_data[CAM_RIGHT] && !past_victims && camera_data[CAM_RIGHT] != 90)) {
      state_data |= BMSK_S_VICTIM_FOUND;
    }

  colour_sensor.update();

  // Serial.print(camera_data[0]); Serial.println(camera_data[1]);

  bno.read();

  // run state machine
  state_update();
}