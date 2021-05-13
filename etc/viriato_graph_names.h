//////////////////////////////////////////
// Node and edges names for Viriato world
//////////////////////////////////////////

#ifndef DSR_NAMES_H
#define DSR_NAMES_H

// NODES
const std::string robot_name = "viriato";
const std::string robot_mind_name = "mind";
const std::string robot_body_name = "body";
const std::string world_name = "world";
const std::string floor_name = "floor";
const std::string viriato_head_camera_pan_tilt = "viriato_head_camera_pan_tilt";
const std::string viriato_head_camera_pan_joint = "viriato_head_camera_pan_joint";
const std::string viriato_head_camera_tilt_joint = "viriato_head_camera_tilt_joint";
const std::string viriato_head_camera_name = "viriato_head_camera_sensor";
const std::string laser_name = "laser";
const std::string glass_1_name= "glass_1";
const std::string collision_box_name= "collision_box";
const std::string viriato_left_arm_tip_name = "viriato_left_arm_tip";

const std::string current_grid_name = "current_grid";
const std::string current_intention_name = "current_intention";
const std::string current_path_name = "current_path";


//NODE TYPES
const std::string path_to_target_type_name = "path_to_target";
const std::string intention_type_name = "intention";
const std::string laser_type_name = "laser";
const std::string omnirobot_type_name = "omnirobot";
const std::string rgbd_type_name = "rgbd";
const std::string left_hand_type_name = "left_hand";
const std::string pan_tilt_type_name = "pan_tilt";
const std::string glass_type_name = "glass";
const std::string cup_type_name = "cup";
const std::string plant_type_name = "plant";
const std::string microwave_type_name = "microwave";
const std::string grid_type_name = "grid";
const std::string room_type_name = "room";
const std::string person_type_name = "person";

// EDGES TYPES
const std::string think_type = "thinks";
const std::string has_type = "has";

#endif
