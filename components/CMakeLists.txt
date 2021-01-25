cmake_minimum_required( VERSION 3.0 )
PROJECT( components )


#include_directories(/usr/include/eigen3)
add_subdirectory( idserver )
add_subdirectory( viriatoDSR )
#add_subdirectory( people_to_dsr )
add_subdirectory( change_graph )
#add_subdirectory( social_navigation )
#add_subdirectory( social_rules )
#add_subdirectory( watcherDSR )
#add_subdirectory( elastic_band )
#add_subdirectory( path_follower )
#add_subdirectory( path_planner_astar )
#add_subdirectory( graspDSR )

add_subdirectory( dsr_tests )

