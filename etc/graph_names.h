//////////////////////////////////////////
// Node and edges names for Viriato world
//////////////////////////////////////////

#ifndef DSR_VIRIATO_NAMES_H
#define DSR_VIRIATO_NAMES_H

#include <string>

// NODES
const std::string robot_name = "robot";
const std::string robot_mind_name = "mind";
const std::string robot_body_name = "body";
const std::string world_name = "world";
const std::string floor_name = "floor";
const std::string viriato_head_camera_pan_tilt_name = "viriato_head_camera_pan_tilt";
const std::string viriato_head_camera_pan_joint_name = "viriato_head_camera_pan_joint";
const std::string viriato_head_camera_tilt_joint_name = "viriato_head_camera_tilt_joint";
const std::string viriato_head_camera_name = "viriato_head_camera_sensor";
const std::string giraff_camera_realsense_name = "giraff_camera_realsense";
const std::string giraff_camera_usb_name = "giraff_camera_usb";
const std::string giraff_camera_face_id_name = "giraff_camera_face_id";
const std::string pioneer_head_camera_left_name = "pioneer_camera_left";
const std::string pioneer_head_camera_right_name = "pioneer_camera_right";
const std::string pioneer_camera_virtual_name = "pioneer_camera_virtual";
const std::string laser_name = "laser";
const std::string laser_social_name = "laser_social";
const std::string battery_name = "battery";
const std::string wifi_name = "wifi";
const std::string ultrasound_name = "ultrasound";
const std::string current_intention_name = "current_intention";
const std::string current_path_name = "current_path";
const std::string viriato_left_arm_tip_name = "viriato_left_arm_tip";
const std::string current_grid_name = "current_grid";

// NODES for OBJECT
const std::string glass_1_name= "glass_1";
const std::string collision_box_name= "collision_box";

//NODE TYPES
const std::string omnirobot_type_name = "omnirobot";
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
const std::string object_type_name = "object";


// EDGES TYPES
const std::string think_type_name = "thinks";
const std::string has_type_name = "has";
const std::string in_type_name = "in";
const std::string attention_action_type_name = "attention_action";
const std::string goto_action_type_name = "goto_action";
const std::string following_action_type_name = "following";


#endif
