//////////////////////////////////////////
// Node and edges names for Pioneer world
//////////////////////////////////////////

#ifndef DSR_NAMES_H
#define DSR_NAMES_H

#include <string>

// NODES
const std::string robot_name = "robot";
const std :: string waypoints_name = "waypoints";
const std::string robot_body_name = "body";
const std::string robot_mind_name = "mind"; 
const std::string world_name = "world"; 
const std::string floor_name = "floor"; 
const std::string pioneer_head_camera_left_name = "pioneer_camera_left"; 
const std::string pioneer_head_camera_right_name = "pioneer_camera_right"; 
const std::string pioneer_camera_virtual_name = "pioneer_camera_virtual"; 
const std::string laser_name = "laser";
const std::string gps_name = "gps";
//const std::string laser_social_name = "laser_social"; 
const std::string battery_name = "battery"; 
const std::string wifi_name = "wifi"; 
const std::string ultrasound_name = "ultrasound"; 
const std::string current_grid_name = "current_grid"; 
const std::string current_intention_name = "current_intention"; 
const std::string current_path_name = "current_path"; 



//NODE TYPES
const std::string omnirobot_type_name = "omnirobot";
const std::string wayp_type_name = "wayp";
const std::string differentialrobot_type_name = "differentialrobot";
const std::string path_to_target_type_name = "path_to_target";
const std::string intention_type_name = "intention";
const std::string personal_space_type_name = "personal_space";
const std::string affordance_space_type_name = "affordance_space";
const std::string laser_type_name = "laser";
const std::string laser_social_type_name = "laser_social";
const std::string rgbd_type_name = "rgbd";
const std::string grid_type_name = "grid";
const std::string left_hand_type_name = "left_hand";
const std::string pan_tilt_type_name = "pan_tilt";
const std::string glass_type_name = "glass";
const std::string cup_type_name = "cup";
const std::string plant_type_name = "plant";
const std::string microwave_type_name = "microwave";
const std::string social_grid_type_name = "social_grid";
const std::string room_type_name = "room";
const std::string person_type_name = "person";
const std::string vase_type_name = "vase";
const std::string refrigerator_type_name = "refrigerator";
const std::string oven_type_name = "oven";
const std::string apple_type_name = "apple";

// EDGES TYPES
const std::string think_type = "thinks";
const std::string has_type = "has";
const std::string in_type_name = "in";
const std::string attention_action_type_name = "attention_action";
const std::string goto_action_type_name = "goto_action";

#endif
