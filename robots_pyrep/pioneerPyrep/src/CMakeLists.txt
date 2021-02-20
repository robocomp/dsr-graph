cmake_minimum_required(VERSION 2.6)
PROJECT(  pioneerPyrep )

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )

ROBOCOMP_IDSL_TO_ICE( CommonBehavior CameraRGBDSimple CameraRGBDSimplePub GenericBase JoystickAdapter Laser LaserPub DifferentialRobot  CoppeliaUtils )

