//////////////////////////////////////////
// Node and edges names for Viriato world
//////////////////////////////////////////

#ifndef DSR_NAMES_H
#define DSR_NAMES_H

// NODES
const std::string robot_name = "omnirobot";
const std::string world_name = "world";
const std::string floor_name = "infiniteFloor";
const std::string viriato_head_camera_pan_tilt = "viriato_head_camera_pan_tilt";
const std::string viriato_head_camera_pan_joint = "viriato_head_camera_pan_joint";
const std::string viriato_head_camera_tilt_joint = "viriato_head_camera_tilt_joint";
const std::string viriato_head_camera_name = "viriato_head_camera_sensor";
const std::string laser_name = "laser";
const std::string glass_1_name= "glass_1";
const std::string collision_box_name= "collision_box";

//NODE TYPES
const std::string path_to_target_type = "path_to_target";
const std::string intention_type = "intention";
const std::string laser_type = "laser";
const std::string omnirobot_type = "omnirobot";


// EDGES TYPES
const std::string think_type = "thinks";
const std::string has_type = "has";

#endif