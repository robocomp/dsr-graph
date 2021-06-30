cmake_minimum_required( VERSION 3.10 )
PROJECT( components )

#add_definitions( -DQT_NO_WARNING_OUTPUT )


#include_directories(/usr/include/eigen3)
add_subdirectory( idserver )
add_subdirectory( viriatoDSR )
add_subdirectory( graph_editor )
#add_subdirectory( people_to_dsr )
#add_subdirectory( change_graph )
#add_subdirectory( graph_editor )
#add_subdirectory( social_navigation )
#add_subdirectory( social_rules )
#add_subdirectory( watcherDSR )
#add_subdirectory( elastic_band )
#add_subdirectory( path_follower )
#add_subdirectory( path_planner_astar )
#add_subdirectory( mission_controller_viriato )

#add_subdirectory( graspDSR )

#add_subdirectory( yolov4-detector )
#add_subdirectory( pan_tilt_attention_control )


add_subdirectory( dsr_tests )

