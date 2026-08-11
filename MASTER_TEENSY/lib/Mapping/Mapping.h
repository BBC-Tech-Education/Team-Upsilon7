#ifndef MAPPING_H
#define MAPPING_H

#include <Config.h>
#include <Arduino.h>
#include <cmath>
#include <vector>
//#include <cstdio>
#include <stdlib.h>
//#include <cassert>
#include <list>
#include <algorithm>
#include <iterator> 
#include <iostream>

struct Coordinates { // Struct for tile coordinates
    int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;
};

struct Tile { // where foward is true 0 degrees
    uint8_t id;
    Coordinates location;
    uint8_t info;
    uint8_t ramp;
    int8_t front;
    int8_t left;
    int8_t right;
    int8_t back ;
};



class Mapping {
public:
    Mapping() {}
    void init();
    
    uint8_t get_current_tileID();
    void set_current_tileID(uint8_t tile_id);
    void switch_current_tile();
    void set_current_info(uint8_t current_info);
    void create_tiles(float bearing, uint8_t tile_available[4]);

    void soft_reset();
    void ramp(float bearing, uint8_t direction, uint8_t num_of_tiles, uint8_t height); // runs at end of ramp
    uint8_t victims();

    void tile_printout(uint8_t tile_id);
    void tile_printout_map(uint8_t tile_id);

    uint8_t black_tile(float bearing); // make it use two bearings
    uint8_t limit_switches_flw(float bearing, uint8_t direction);
    uint8_t follow_left_wall(float bearing); // finds next spot l,f,r,b
    uint8_t mapping_alg(float bearing);

private:
    Tile* map;
    uint8_t tile_num = 1;
    uint8_t current_tile_id = 0;
    int8_t next_tile_id = -1; 
    std::vector<int8_t> past_tiles = {0};
    std::vector<int8_t> past_tiles_mapping = {0};
    std::vector<int8_t> branches = {0};
    bool tile_visted = false;

    void create_empty_tile(uint8_t tile_id);
    float closest_bearing(float bearing); 
    std::array<uint8_t, 4> convert_true(float bearing, uint8_t tile_available[4]);
    std::array<int8_t, 4> convert_bering(float bearing, int8_t tile_available[4]);

    float tile_headings[4] = {-90.0, 0.0, 90.0, 180.0};

    //float tile_headings[5] = {-180.0, -90.0, 0.0, 90.0, 180.0};


    bool black_tile_here[0];

    /*
    //Setting the value
    info |= BMSK_VICTIM;

    // Clearing the value
    info &= ~BMSK_VICTIM

    // Reading the value
    reading = info & BMSK_VICTIM
    */
};
#endif