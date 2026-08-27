#ifndef CONFIG_H
#define CONFIG_H


// ================================== DEBUG ==================================
// ----------------- General -----------------
#define PRINTS_GENERAL 1

// ----------------- Colour -----------------
#define PRINTS_ALL_COLOUR 0
#define PRINTS_COLOUR_DATA 0

#define DEBUG_COLOUR_DATA (PRINTS_COLOUR_DATA || PRINTS_GENERAL|| PRINTS_ALL_COLOUR)

// ----------------- Camera -----------------
#define PRINTS_ALL_CAMERA 0
#define PRINTS_CAMERA_DATA 0

#define DEBUG_CAMERA_DATA (PRINTS_CAMERA_DATA || PRINTS_GENERAL|| PRINTS_ALL_CAMERA)


// --------------- LRFS ---------------------
#define PRINTS_ALL_LRF 0
#define PRINTS_LRF_DATA 0
#define PRINTS_TARGET_DISTANCES 0

#define DEBUG_LRF_DATA (PRINTS_LRF_DATA || PRINTS_GENERAL || PRINTS_ALL_LRF)
#define DEBUG_TARGET_DISTANCES (PRINTS_TARGET_DISTANCES || PRINTS_ALL_LRF || PRINTS_GENERAL)


// ----------------- BNO -----------------
#define PRINTS_ALL_BNO 0
#define PRINTS_BNO_DATA 0

#define DEBUG_BNO_DATA (PRINTS_BNO_DATA || PRINTS_GENERAL|| PRINTS_ALL_BNO)


// ----------------- States -----------------
#define PRINTS_ALL_STATE 0
#define PRINTS_STATE_DATA 0

#define DEBUG_STATE_DATA (PRINTS_STATE_DATA || PRINTS_GENERAL || PRINTS_ALL_STATE)


// ----------------- Mapping -----------------
#define PRINTS_ALL_MAPPING 0

#define PRINTS_CREATE_TILE 0
#define PRINTS_CURRENT_TILE 0
#define PRINTS_MAP 0
#define PRINTS_DIRECTIONS 0
#define PRINTS_ALG 0
#define PRINTS_RAMP 0

#define PRINTS_RAMP_LENGTH 0
#define PRINTS_BLACK_TILE 0

#define DEBUG_CREATE_TILE (PRINTS_CREATE_TILE || PRINTS_ALL_MAPPING)
#define DEBUG_CURRENT_TILE (PRINTS_CURRENT_TILE || PRINTS_ALL_MAPPING)
#define DEBUG_MAP (PRINTS_MAP || PRINTS_ALL_MAPPING)
#define DEGUB_DIRECTIONS (PRINTS_DIRECTIONS || PRINTS_ALL_MAPPING)
#define DEBUG_ALG (PRINTS_ALG || PRINTS_ALL_MAPPING)
#define DEBUG_RAMP (PRINTS_RAMP || PRINTS_ALL_MAPPING)
#define DEBUG_BLACK_TILE (PRINTS_BLACK_TILE || PRINTS_ALL_MAPPING)

#define DEBUG_RAMP_LENGTH (PRINTS_RAMP_LENGTH || PRINTS_ALL_MAPPING)




// ================================= BIT MASKS =================================
// ------------ Tile Infomation ------------
#define BMSK_T_VICTIM   (1 << 0) // 0000 0001
#define BMSK_T_EXPLORED (1 << 1) // 0000 0010
#define BMSK_T_BLACK    (1 << 2) // 0000 0100
#define BMSK_T_SILVER   (1 << 3) // 0000 1000
#define BMSK_T_BLUE     (1 << 4) // 0001 0000 
#define BMSK_T_RED      (1 << 5) // 0010 0000 
#define BMSK_T_OBSTACLE (1 << 6) // 0100 0000 
#define BMSK_T_RAMP     (1 << 7) // 1000 0000


// ------------ Ramp Infomation ------------
#define BMSK_R_RAMP_DOWN      (1 << 0)
#define BMSK_R_RAMP_UP        (1 << 1)
#define BMSK_R_RAMP_PART_DOWN (1 << 2)
#define BMSK_R_RAMP_PART_UP   (1 << 3)


// ------------ Limit Switches ------------
#define BMSK_L_LIMIT    (1 << 0)
#define BMSK_L_LIMIT_FL (1 << 1)
#define BMSK_L_LIMIT_FR (1 << 2)
#define BMSK_L_LIMIT_BL (1 << 3)
#define BMSK_L_LIMIT_RL (1 << 4)

// ------------ State Infomation ------------
#define BMSK_S_NEW_STATE         (1 << 0)
#define BMSK_S_VICTIM_FOUND      (1 << 1)
#define BMSK_S_BLUE_TILE_DONE    (1 << 2)
#define BMSK_S_MAPPING_COMPLETED (1 << 3)
#define BMSK_S_VICTIM_COMPLETED  (1 << 4)
#define BMSK_S_SWITCH_TILE       (1 << 5)
#define BMSK_S_MAP_RESETED       (1 << 6)
#define BMSK_S_RESETING          (1 << 7)




// ================================== SPEEDS ==================================
#define FORWARD_SPEED 100
#define TURN_SPEED 100
#define BACK_SPEED 70
#define TILE_TIME (2800/300) // ms @100 * 1.2




// ================================ DIMENSIONS ================================
#define TILE_WIDTH 300
#define ROBOT_LENGTH 155 // mm
#define ROBOT_WIDTH 107 // mm
#define ROBOT_HEIGHT 120 // mm
#define ROBOT_LRFCX 39 // mm and fix




// ================================ CONSTANTS ================================
// ----------------- PID Constants -----------------
#define PID_DIS 200
// ----------------- LRF Constants -----------------
#define LRF_NUM       10
#define LRF_SHORT_NUM 8
#define LRF_LONG_NUM  2
#define LRF_SHORT_MAX_DIST 1200
#define LRF_LONG_MAX_DIST 2000
#define LRF_SHORT_INVALID_DIST 10
#define LRF_LONG_INVALID_DIST 600

#define LRF_FRONT_SHORT_SIDE 0
#define LRF_LEFT_SIDE 1
#define LRF_RIGHT_SIDE 2
#define LRF_BACK_SIDE 3
#define LRF_FRONT_LONG_SIDE 4


// ----------------- Colour Constants -----------------
#define COLOUR_NUM 4
#define BLACK 0
#define BLUE 1
#define SILVER 2
#define RED 3


// ----------------- Ramp Constants -----------------
#define RAMP_TOL_UP 10
#define RAMP_TOL_DOWN 6

#define RAMP_DIR      0
#define RAMP_DIS      1
#define RAMP_CUR_ANG  2
#define RAMP_DIS_READ 3


// ----------------- Ramp Constants -----------------
#define ANGLE_TOL 2


// ----------------- Obstacles Constants -----------------
#define OBS_OFFSET 5


// ----------------- Camera Constants -----------------
#define CAM_LEFT  0
#define CAM_RIGHT 1


// ----------------- Movement Constants -----------------
#define MOVEMENT_OFFSET 20




#endif