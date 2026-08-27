#include "Mapping.h"


void Mapping::init()
{
    map = (Tile*)malloc(sizeof(Tile) * tile_num);
    current_tile_id = 0;
    create_empty_tile(current_tile_id);
}



uint8_t Mapping::get_current_tileID() // returns current tile id
{
    return current_tile_id;
}



void Mapping::set_current_tileID(uint8_t tile_id){
    current_tile_id = tile_id;
}



void Mapping::switch_current_tile() // changes the tile; for when robot physial moves tiles
{
    Serial.println("I HAVE THE SWITCHHHHHHHHHHHHHH!!!!!");
    (map[current_tile_id]).info |= BMSK_T_EXPLORED;
    past_tiles.push_back(next_tile_id);

    if (!(map[next_tile_id].info & BMSK_T_EXPLORED)) {
        Serial.println("I HAVE THE SWITCHESSSSSSSSSSS!!!!!");
        past_tiles_mapping.push_back(next_tile_id);
    }

    current_tile_id = next_tile_id;
}



void Mapping::set_current_info(uint8_t current_info) // add the current info for the tile
{
    (map[current_tile_id]).info |= current_info;
}



void Mapping::create_tiles(float bearing, uint8_t tile_available[4])
{
    // 1. ++++++++ What way is the robot facing ++++++++
    float closest_heading = closest_bearing(bearing);
    Serial.print("Closest Bearing:"); Serial.println(closest_heading); 

    // 2. ++++++++ Where are the connected tiles based on this ++++++++
    std::array<uint8_t, 4> connected_tile = convert_true(bearing, tile_available); // at 0 degrees: left, front, right, back


    #if DEBUG_CREATE_TILE
    Serial.print("Connected Tiles:"); Serial.print(connected_tile[0]); Serial.print("\t");
    Serial.print(connected_tile[1]); Serial.print("\t"); Serial.print(connected_tile[2]); Serial.print("\t");
    Serial.println(connected_tile[3]);
    #endif

    // 3. ++++++++ What are the coords are these tiles ++++++++
    int8_t current_x = (map[current_tile_id]).location.x;
    int8_t current_y = (map[current_tile_id]).location.y;
    int8_t current_z = (map[current_tile_id]).location.z;

    int8_t coord_values[4][2] = {
        {(int8_t)(current_x - 1), current_y              },
        {current_x              , (int8_t)(current_y + 1)},
        {(int8_t)(current_x + 1), current_y              },
        {current_x              , (int8_t)(current_y - 1)}
    };

    #if DEBUG_CREATE_TILE
    Serial.print("X Value :\t"); Serial.print(coord_values[0][0]); Serial.print("\t");
    Serial.print(coord_values[1][0]); Serial.print("\t"); Serial.print(coord_values[2][0]); Serial.print("\t");
    Serial.println(coord_values[3][0]);

    Serial.print("Y Value :\t"); Serial.print(coord_values[0][1]); Serial.print("\t");
    Serial.print(coord_values[1][1]); Serial.print("\t"); Serial.print(coord_values[2][1]); Serial.print("\t");
    Serial.println(coord_values[3][1]);
    #endif

    // 4. ++++++++ Have i made these tiles ++++++++
    uint8_t uncommon_tiles[4] = {
        connected_tile[0],
        connected_tile[1], 
        connected_tile[2],
        connected_tile[3]
    };
    
    // check for all directions l,f,r,b
    for (uint8_t i = 0; i < 4; i++) {   
        uint8_t common_tile = 0;
        int8_t x_coord_1 = coord_values[i][0];
        int8_t y_coord_1 = coord_values[i][1];

         // check every tile that has been made
        for (uint8_t j = 0; j < tile_num; j++) {
            int8_t x_coord_2 = (map[j]).location.x;
            int8_t y_coord_2 = (map[j]).location.y;
            int8_t z_coord_2 = (map[j]).location.z;


            if (x_coord_1 == x_coord_2 && y_coord_1 == y_coord_2) {
                //common_tile = 1;
                if(current_z == z_coord_2) {
                    common_tile = 1;
                } else if (map[j].info & BMSK_T_RAMP) {
                    common_tile = 1;
                }
            }
        }

        // if has been made make uncommon tile false
        if (common_tile) {
            uncommon_tiles[i] = 0;
        }    
    }

    #if DEBUG_CREATE_TILE
    Serial.print("Uncommon tiles :"); Serial.print(uncommon_tiles[0]); Serial.print("\t");
    Serial.print(uncommon_tiles[1]); Serial.print("\t"); Serial.print(uncommon_tiles[2]); Serial.print("\t");
    Serial.println(uncommon_tiles[3]);
    #endif

    // 5. ++++++++ Make new tiles ++++++++
    uint8_t num_of_tile_to_create = 0;
    for (uint8_t i = 0; i < 4; i++) {
        num_of_tile_to_create += uncommon_tiles[i];
    }
    
    // Save the old count so we know where the new tiles start
    uint8_t old_tile_num = tile_num; 
    tile_num += num_of_tile_to_create;
    
    // Resize the map array
    map = (Tile*)realloc(map, sizeof(Tile) * tile_num);

    // Create only the brand new tiles
    for (int8_t i = old_tile_num; i < tile_num; i++) {
        create_empty_tile(i);
    }
    
    // Track the absolute map index for all 4 directions
    int8_t assigned_index[4] = {-1, -1, -1, -1}; // -1 means no tile in this direction
    int8_t new_tile_counter = 0;

    for (int8_t i = 0; i < 4; i++) {

        // it's a completely new tile'
        if (uncommon_tiles[i]) {

            // set coordinates and save the index
            uint8_t target_index = old_tile_num + new_tile_counter;
            map[target_index].location.x = coord_values[i][0];
            map[target_index].location.y = coord_values[i][1];
            map[target_index].location.z = current_z;
            assigned_index[i] = target_index;
            new_tile_counter++;

        // The tile was already made
        } else if (connected_tile[i]) {
            
            // search the map to find its existing index
            for (uint8_t j = 0; j < old_tile_num; j++) { // will need changing
                if (map[j].location.x == coord_values[i][0] && map[j].location.y == coord_values[i][1]) {
                    assigned_index[i] = j; // Grab the existing tile's index
                    break;
                }
            }
        }
    }


    // 6. ++++++++ Pointer to connected tiles ++++++++
    #if DEBUG_CREATE_TILE
    Serial.print(assigned_index[0]);Serial.print(assigned_index[1]);
    Serial.print(assigned_index[2]);Serial.println(assigned_index[3]);
    #endif

    // Connect the current tile to its neighbors (whether they are new or old)
    map[current_tile_id].left  = (assigned_index[0]);
    map[current_tile_id].front = (assigned_index[1]);
    map[current_tile_id].right = (assigned_index[2]);
    map[current_tile_id].back  = (assigned_index[3]);

    // Connect neighbors back to the current tile (mutual linking)
    // 0 (Left) connects back via Right
    if (assigned_index[0] != -1) {
        map[assigned_index[0]].right = current_tile_id;
    }
    // 1 (Front) connects back via Back
    if (assigned_index[1] != -1) {
        map[assigned_index[1]].back = current_tile_id;
    }
    // 2 (Right) connects back via Left
    if (assigned_index[2] != -1) {
        map[assigned_index[2]].left = current_tile_id;
    }
    // 3 (Back) connects back via Front
    if (assigned_index[3] != -1) {
        map[assigned_index[3]].front = current_tile_id;
    }

    #if DEBUG_CURRENT_TILE
    tile_printout(current_tile_id);
    #endif

    #if DEBUG_MAP
    Serial.println("MAP DATA");
    for (size_t i = 0; i <= current_tile_id; i++) {
        tile_printout_map(i);
    }
    #endif
    
}



void Mapping::create_empty_tile(uint8_t tile_id) // Fill out the basic infomation for a tile
{
    (map[tile_id]).id = tile_id;
    (map[tile_id]).left = -1;
    (map[tile_id]).front = -1;
    (map[tile_id]).right = -1;
    (map[tile_id]).back = -1;
    (map[tile_id]).info = 0;
    (map[tile_id]).location.x = 0;
    (map[tile_id]).location.y = 0;
    (map[tile_id]).location.z = 0;
}



float Mapping::closest_bearing(float bearing) 
{ 
    // finds the closest 90* beaing to what the robot is currently facing
    float closest_heading = tile_headings[0];
    float diff = bearing - tile_headings[0];
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }

    float min_difference = fabs(diff);
    
    for (int i = 1; i < 4; i++) {
        diff = bearing - tile_headings[i];
        if (diff > 180.0f) {
            diff -= 360.0f;
        } else if (diff < -180.0f) {
            diff += 360.0f;
        }
        float current_difference = fabs(diff);
        if (current_difference < min_difference) {
            min_difference = current_difference;
            closest_heading = tile_headings[i];
        }
    }
    return closest_heading;
}



std::array<uint8_t, 4> Mapping::convert_true(float bearing, uint8_t tile_available[4])
{
    // converts the tiles infomation from relasitve to true
    float closest_heading = closest_bearing(bearing);
    std::array<uint8_t, 4> connected_tile = {0, 0, 0, 0}; // at 0 degrees: left, front, right, back

    if (closest_heading == 0) { // 0 degrees
        connected_tile[0] = tile_available[0];   connected_tile[1] = tile_available[1];
        connected_tile[2] = tile_available[2];  connected_tile[3] = tile_available[3];

    } else if (closest_heading == 90) { // 90 degrees
        connected_tile[0] = tile_available[3];   connected_tile[1] = tile_available[0];
        connected_tile[2] = tile_available[1];  connected_tile[3] = tile_available[2];

    } else if (closest_heading == 180) { // 180 degrees
        connected_tile[0] = tile_available[2];  connected_tile[1] = tile_available[3];
        connected_tile[2] = tile_available[0];   connected_tile[3] = tile_available[1];

    } else { // 270 degrees
        connected_tile[0] = tile_available[1];  connected_tile[1] = tile_available[2];
        connected_tile[2] = tile_available[3];   connected_tile[3] = tile_available[0];
    }
    return connected_tile;
}



std::array<int8_t, 4> Mapping::convert_bering(float bearing, int8_t tile_available[4])
{
    // converts the tiles infomation from true to relative
    float closest_heading = closest_bearing(bearing);
    std::array<int8_t, 4> connected_tile = {-1, -1, -1, -1}; // at 0 degrees: left, front, right, back
    if (closest_heading == 0) { // 0 degrees
        connected_tile[0] = tile_available[0]; connected_tile[1] = tile_available[1];
        connected_tile[2] = tile_available[2]; connected_tile[3] = tile_available[3];

    } else if (closest_heading == 90) { // 90 degrees
        connected_tile[0] = tile_available[1]; connected_tile[1] = tile_available[2];
        connected_tile[2] = tile_available[3]; connected_tile[3] = tile_available[0];


    } else if (closest_heading == 180) { // 180 degrees
        connected_tile[0] = tile_available[2]; connected_tile[1] = tile_available[3];
        connected_tile[2] = tile_available[0]; connected_tile[3] = tile_available[1];

    } else { // -90 degrees
        connected_tile[0] = tile_available[3]; connected_tile[1] = tile_available[0];
        connected_tile[2] = tile_available[1]; connected_tile[3] = tile_available[2];
    }
    return connected_tile;
}



void Mapping::soft_reset() 
{
    for (uint8_t i = (past_tiles.size()-1); i >= 0; i--)
    {   
       uint8_t  tile_id = past_tiles[i];
        if(map[tile_id].info & BMSK_T_SILVER) {
            current_tile_id = tile_id;
            break;
        }
        map[tile_id].info&= ~BMSK_T_EXPLORED;
        past_tiles.pop_back();

        //does mapping need anything

        // when 0 that is the current id
        
        
        
        
        
        // uint8_t tile_id = 0;
        // if (past_tiles.size() < i) {
        //     current_tile_id = 0;
        //     break;
        // } else if(i >= 0 && i <= 255) {
        //     tile_id = past_tiles[i];
        // }



        // if(map[tile_id].info & BMSK_T_SILVER) {
        //     current_tile_id = tile_id;
        //     break;
        // } else {
        //     map[tile_id].info&= ~BMSK_T_EXPLORED;
        // }
        
    }
    tile_printout(current_tile_id);
}



void Mapping::ramp(float bearing, uint8_t direction, uint8_t num_of_tiles, uint8_t height)
{
    // for when the tile has finished the ramp

    // how do i know if it is stairs that go across one tile????

    // sets the current tile as a ramp tile and look at what way it goes
    // makes the new tile below or above
    switch_current_tile();
    if (!(map[current_tile_id].info & BMSK_T_EXPLORED)) {
        // DO I GET HERE????????????
        uint8_t height_change[num_of_tiles]; // make all 0
        
        if(height == 0) {
            height = 1;
        }

        if(num_of_tiles == 1) {
            height_change[0] = height;
        } else if (num_of_tiles == 2) {
            height_change[0] = 0;
            height_change[1] = height;
        }else {
            uint8_t halfway = 0;

            if ((height % 2) == 0) {
                halfway = uint8_t(num_of_tiles/2) + 1;

            } else {
                halfway = uint8_t(num_of_tiles/2);
            }

            uint8_t changed = 0;
            for (size_t i = 0; i < num_of_tiles; i++)
            {
                if((i+1) == (halfway)) {
                    height_change[i] = height;
                    changed = 1;
                } else {
                    height_change[i] = 0;
                }
            }  
        }
        Serial.print("Hellooooooooo"); Serial.println(num_of_tiles);
        for (uint8_t i = 0; i < num_of_tiles; i++)
        {
            map[current_tile_id].info |= BMSK_T_RAMP; 
            if (direction == 2) { // for it the ramp goes down
                map[current_tile_id].ramp |= BMSK_R_RAMP_DOWN;
                if (num_of_tiles > 1) {
                    map[current_tile_id].ramp |= BMSK_R_RAMP_PART_DOWN;
                    map[current_tile_id].location.z -= height_change[i];
                } else {
                    map[current_tile_id].location.z -= height;
                }
            

            } else { // for it the ramp goes up
                map[current_tile_id].ramp |= BMSK_R_RAMP_UP;
                if (num_of_tiles > 1) {
                    map[current_tile_id].ramp |= BMSK_R_RAMP_PART_UP;
                    map[current_tile_id].location.z += height_change[i];
                     
                } else {
                    map[current_tile_id].location.z += height;
                }  
            }
            
            // runs create tile for the new tile with adjusted z value
            uint8_t tile_available[4] = {0,1,0,1};
            create_tiles(bearing, tile_available); // will need fixing
            follow_left_wall(bearing);
            switch_current_tile();
        }

    } else {
        for (uint8_t i = 0; i < num_of_tiles; i++) 
        {
            uint8_t tile_available[4] = {0,1,0,1};
            create_tiles(bearing, tile_available); // will need fixing
            switch_current_tile();
        }
    }

    #if DEBUG_RAMP
    tile_printout(past_tiles[past_tiles.size()-2]);
    tile_printout(current_tile_id);
    #endif
    
}



uint8_t Mapping::victims(){
    return (map[current_tile_id].info & BMSK_T_VICTIM);
}



void Mapping::tile_printout(uint8_t tile_id) // prints out all the tiles data
{
    Serial.print(" ------- MAPPING INFOMATION FOR "); 
    Serial.print(tile_id); Serial.println(" -------");

    Serial.print("Tile ID: \t"); Serial.println(map[tile_id].id);

    Serial.print("X Coordinate: \t"); Serial.println(map[tile_id].location.x);

    Serial.print("Y Coordinate: \t"); Serial.println(map[tile_id].location.y);

    Serial.print("Z Coordinate: \t"); Serial.println(map[tile_id].location.z);

    Serial.print("Left Tile: \t");  
    if (map[tile_id].left >= 0) { Serial.println(map[(map[tile_id].left)].id); 
    } else { Serial.println("WALL"); }

    Serial.print("Front Tile: \t"); 
    if (map[tile_id].front >= 0) { Serial.println(map[(map[tile_id].front)].id); 
    } else { Serial.println("WALL"); }

    Serial.print("Right Tile: \t"); 
    if (map[tile_id].right >= 0) { Serial.println(map[(map[tile_id].right)].id);
    } else { Serial.println("WALL"); }

    Serial.print("Back Tile: \t");  
    if (map[tile_id].back >= 0) { Serial.println(map[(map[tile_id].back)].id);
    } else { Serial.println("WALL"); }

    Serial.print("Info \t \t"); Serial.println(map[tile_id].info);

    Serial.print("Victim: \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_VICTIM);

    Serial.print("Explored: \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_EXPLORED);

    Serial.print("Black Tile: \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_BLACK);

    Serial.print("Silver Tile: \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_SILVER);

    Serial.print("Blue_tile: \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_BLUE);

    Serial.print("Red_tile: \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_RED);

    Serial.print("Obstacle: \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_OBSTACLE);

    Serial.print("RAMP: \t \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_RAMP);

    Serial.print("RAMP UP: \t"); 
    Serial.println((map[tile_id].ramp) & BMSK_R_RAMP_UP);

    Serial.print("RAMP DOWN: \t"); 
    Serial.println((map[tile_id].ramp) & BMSK_R_RAMP_DOWN);

    Serial.print("RAMP PART UP: \t"); 
    Serial.println((map[tile_id].ramp) & BMSK_R_RAMP_PART_UP);

    Serial.print("RAMP PART DOWN: "); 
    Serial.println((map[tile_id].ramp) & BMSK_R_RAMP_PART_DOWN);
}



void Mapping::tile_printout_map(uint8_t tile_id) // prints out all the tiles data
{
    Serial.print(" +++++++++++ MAP DATA FOR "); 
    Serial.print(tile_id); Serial.println(" +++++++++++ ");

    Serial.print("Tile ID: \t"); Serial.println(map[tile_id].id);

    Serial.print("X Coordinate: \t"); Serial.println(map[tile_id].location.x);

    Serial.print("Y Coordinate: \t"); Serial.println(map[tile_id].location.y);

    Serial.print("Z Coordinate: \t"); Serial.println(map[tile_id].location.z);

    Serial.print("Left Tile: \t");  
    if (map[tile_id].left >= 0) { Serial.println(map[(map[tile_id].left)].id); 
    } else { Serial.println("WALL"); }

    Serial.print("Front Tile: \t"); 
    if (map[tile_id].front >= 0) { Serial.println(map[(map[tile_id].front)].id); 
    } else { Serial.println("WALL"); }

    Serial.print("Right Tile: \t"); 
    if (map[tile_id].right >= 0) { Serial.println(map[(map[tile_id].right)].id);
    } else { Serial.println("WALL"); }

    Serial.print("Back Tile: \t");  
    if (map[tile_id].back >= 0) { Serial.println(map[(map[tile_id].back)].id);
    } else { Serial.println("WALL"); }

    Serial.print("RAMP: \t \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_RAMP);

    Serial.print("Silver Tile: \t"); 
    Serial.println((map[tile_id].info) & BMSK_T_SILVER);

    Serial.println(" +++++++++++++++++++++++++++++++++ "); 
    Serial.println(" "); 

    if(tile_id == 0) {
        Serial.println(map[(map[tile_id].front)].id);
    }
}



uint8_t Mapping::black_tile(float bearing) 
{   // When the robot is on a black tile
    Serial.println("BLACK TILE START");
    for (uint8_t i = 0; i < past_tiles_mapping.size(); i++)
        {
            Serial.print(past_tiles_mapping[i]); Serial.print(" ");
        }
    Serial.println();

    (map[next_tile_id]).info |= BMSK_T_EXPLORED;
    map[next_tile_id].info |= BMSK_T_BLACK;

    if(current_tile_id == next_tile_id) {
        past_tiles.pop_back();
        past_tiles_mapping.pop_back();
        current_tile_id = past_tiles_mapping[past_tiles_mapping.size()-1];  
    } else {
        (map[current_tile_id]).info |= BMSK_T_EXPLORED;
    }
    for (uint8_t i = 0; i < past_tiles_mapping.size(); i++)
        {
            Serial.print(past_tiles_mapping[i]); Serial.print(" ");
        }
    Serial.println();
    Serial.println("BLACK TILE END");
    return mapping_alg(bearing);
}



uint8_t Mapping::limit_switches_flw(float bearing, uint8_t direction) 
{
    next_tile_id = -1;
    // find out what way the robot is facing
    float closest_heading = tile_headings[0];
    float diff = bearing - tile_headings[0];
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }

    float min_difference = fabs(diff);
    
    for (int i = 1; i < 4; i++) {
        diff = bearing - tile_headings[i];
        if (diff > 180.0f) {
            diff -= 360.0f;
        } else if (diff < -180.0f) {
            diff += 360.0f;
        }
        float current_difference = fabs(diff);
        if (current_difference < min_difference) {
            min_difference = current_difference;
            closest_heading = tile_headings[i];
        }
    }
    
    // then do connected tiles so it is for a heading o f 0*
    int8_t connected_tile[4] = {-1,-1,-1,-1}; // at 0 degrees: left, front, right, back

    int8_t current_tiles[4] = {
        ((int8_t)(map[current_tile_id].left)), 
        ((int8_t)(map[current_tile_id].front)),
        ((int8_t)(map[current_tile_id].right)), 
        ((int8_t)(map[current_tile_id].back))};
    

    for (int8_t i = 0; i < 4; i++)
    {
        if((direction-1) == i) {
            current_tiles[i] = 0;
        }
    }


    for (int8_t i = 0; i < 4; i++)
    {
        if (current_tiles[i] >= 0) {
            if (map[current_tiles[i]].info & BMSK_T_BLACK) {
                current_tiles[i] = -1;
                #if DEBUG_BLACK_TILE
                Serial.println("DO I GET HERE???????");
                #endif
            }
        }
    }

    if ((closest_heading == -360) || (closest_heading == 0) || (closest_heading == 360)) { // 0 degrees
        connected_tile[0] = current_tiles[0]; connected_tile[1] = current_tiles[1];
        connected_tile[2] = current_tiles[2]; connected_tile[3] = current_tiles[3];

    } else if (closest_heading == 90 || closest_heading == -270) { // 90 degrees
        connected_tile[0] = current_tiles[1]; connected_tile[1] = current_tiles[2];
        connected_tile[2] = current_tiles[3]; connected_tile[3] = current_tiles[0];


    } else if (closest_heading == 180 || closest_heading == -180) { // 180 degrees
        connected_tile[0] = current_tiles[2]; connected_tile[1] = current_tiles[3];
        connected_tile[2] = current_tiles[0]; connected_tile[3] = current_tiles[1];

    } else { // 270 degrees
        connected_tile[0] = current_tiles[3]; connected_tile[1] = current_tiles[0];
        connected_tile[2] = current_tiles[1]; connected_tile[3] = current_tiles[2];
    }
    #if DEGUB_DIRECTIONS
    Serial.print("Connnect Tile: "); Serial.print(connected_tile[0]); Serial.print("\t");
    Serial.print(connected_tile[1]); Serial.print("\t"); Serial.print(connected_tile[2]); 
    Serial.print("\t"); Serial.println(connected_tile[3]); 
    #endif

    if (connected_tile[0] >= 0) {
        // record where will move next
        next_tile_id = connected_tile[0];
        Serial.print("Next Tile ID"); Serial.println(next_tile_id);
        return 1;

    } else if (connected_tile[1] >= 0) { 
        // record where will move next
        next_tile_id = connected_tile[1];
        Serial.print("Next Tile ID"); Serial.println(next_tile_id);
        return 2;

    } else if (connected_tile[2] >= 0) {
        // record where will move next
        next_tile_id = connected_tile[2];
        Serial.print("Next Tile ID"); Serial.println(next_tile_id);
        return 3;

    } else {
        // record where will move next
        next_tile_id = connected_tile[3];
        Serial.print("Next Tile ID"); Serial.println(next_tile_id);
        return 4;
    }
}



uint8_t Mapping::follow_left_wall(float bearing)
{
    next_tile_id = -1;
    // find out what way the robot is facing
    float closest_heading = closest_bearing(bearing);
    
    // then do connected tiles so it is for a heading of 0 degrees
     // at 0 degrees: left, front, right, back

    int8_t current_tiles[4] = {
        ((int8_t)(map[current_tile_id].left)), 
        ((int8_t)(map[current_tile_id].front)),
        ((int8_t)(map[current_tile_id].right)), 
        ((int8_t)(map[current_tile_id].back))};
    
    for (int8_t i = 0; i < 4; i++) {
        if (current_tiles[i] >= 0) {
            if (map[current_tiles[i]].info & (BMSK_T_BLACK | BMSK_T_EXPLORED)) {
                current_tiles[i] = -1;
                #if DEBUG_BLACK_TILE
                Serial.println("DO I GET HERE???????");
                #endif
            }
        }
    }
    std::array<int8_t, 4> connected_tile = convert_bering(bearing, current_tiles);

    #if DEGUB_DIRECTIONS
    Serial.print("Connnect Tile: "); Serial.print(connected_tile[0]); Serial.print("\t");
    Serial.print(connected_tile[1]); Serial.print("\t"); Serial.print(connected_tile[2]); 
    Serial.print("\t"); Serial.println(connected_tile[3]); 
    #endif

    if (connected_tile[0] >= 0) {
        // record where will move next
        next_tile_id = connected_tile[0];
        Serial.print("Next Tile ID"); Serial.println(next_tile_id);
        return 1;

    } else if (connected_tile[1] >= 0) { 
        // record where will move next
        next_tile_id = connected_tile[1];
        Serial.print("Next Tile ID"); Serial.println(next_tile_id);
        return 2;

    } else if (connected_tile[2] >= 0) {
        // record where will move next
        next_tile_id = connected_tile[2];
        Serial.print("Next Tile ID"); Serial.println(next_tile_id);
        return 3;

    } else {
        // record where will move next
        next_tile_id = connected_tile[3];
        Serial.print("Next Tile ID"); Serial.println(next_tile_id);
        return 4;
    }
}



uint8_t Mapping::mapping_alg(float bearing) 
{
    tile_visted = false;

    uint8_t direction_value = follow_left_wall(bearing);

    if(!(map[current_tile_id].info & BMSK_T_EXPLORED)) { // have not been here
        if (direction_value != 4 || next_tile_id != -1) {
            return direction_value;
        }
    }
    
    // have been here
    for (uint8_t i = 0; i < past_tiles_mapping.size(); i++)
        {
            Serial.print(past_tiles_mapping[i]); Serial.print(" ");
        }
    Serial.println();

    past_tiles_mapping.pop_back();

    if (direction_value == 4) {
        // Back track
        Serial.println("BACKTRACKING");
        next_tile_id = past_tiles_mapping[past_tiles_mapping.size() - 1];

        Serial.print(current_tile_id); Serial.print(" "); Serial.println(next_tile_id);

        for (uint8_t i = 0; i < past_tiles_mapping.size(); i++)
        {
            Serial.print(past_tiles_mapping[i]); Serial.print(" ");
        }
        Serial.println();

        if (next_tile_id == map[current_tile_id].left) {
            direction_value = 1;
        } else if (next_tile_id == map[current_tile_id].front) {
            direction_value = 2;
        } else if (next_tile_id == map[current_tile_id].right) {
            direction_value = 3;
        } else {
            direction_value = 4;
        }


        Serial.printf("ABSOLUTE NEXT TILE: %d\n", direction_value);

        float closest_heading = closest_bearing(bearing);

        Serial.print("Closest Heading: ");
        Serial.println(closest_heading);

        if (closest_heading == -90.0f) {
            direction_value += 1;
        } else if (closest_heading == 90.0f) {
            direction_value -= 1;
        } else if (closest_heading == 180.0f) {
            direction_value -= 2;
        }

        Serial.print("Raw direction_value: ");
        Serial.println(direction_value);

        if (direction_value > 4) {
            direction_value -= 4;
        } else if (direction_value < 1) {
            direction_value += 4;
        }

        Serial.print("FINAL direction_value: ");
        Serial.println(direction_value);
    }

    Serial.print(current_tile_id); Serial.print(" "); Serial.println(next_tile_id);

    return direction_value;
}