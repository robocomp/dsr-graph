cmake_minimum_required(VERSION 2.6)
PROJECT(  human_social_spaces_dsr )

INCLUDE( /opt/robocomp/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior HumanToDSR )
