#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from genericworker import *
import os
from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.mobiles.youbot import YouBot
from pyrep.objects.vision_sensor import VisionSensor
import numpy as np
import cv2

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 100
       
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        
        SCENE_FILE = '/home/robocomp/software/vrep-simulator/viriato/alab-completo-nolaser.ttt'
        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        
        self.camera = VisionSensor("real_sense_sensor")
        self.laser = VisionSensor("laser")
        self.timer.start(self.Period)


    @QtCore.Slot()
    def compute(self):
        
        self.pr.step()
        self.img = camera.capture_rgb()
        self.laser  = camera.capture_depth()
        print("gola")

    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):
        #
        #subscribesToCODE
        #
        pass


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self):
        #
        # implementCODE
        #
        im = TImage()
        dep = TDepth()
        return [im, dep]


    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self):
        #
        # implementCODE
        #
        dep = TDepth()
        return dep


    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self):
        #
        # implementCODE
        #
        im = TImage()
        return im


    #
    # getLaserAndBStateData
    #
    def Laser_getLaserAndBStateData(self):
        ret = TLaserData()
        #
        # implementCODE
        #
        bState = RoboCompGenericBase.TBaseState()
        return [ret, bState]


    #
    # getLaserConfData
    #
    def Laser_getLaserConfData(self):
        ret = LaserConfData()
        #
        # implementCODE
        #
        return ret


    #
    # getLaserData
    #
    def Laser_getLaserData(self):
        ret = TLaserData()
        #
        # implementCODE
        #
        return ret


    #
    # correctOdometer
    #
    def OmniRobot_correctOdometer(self, x, z, alpha):
        #
        # implementCODE
        #
        pass


    #
    # getBasePose
    #
    def OmniRobot_getBasePose(self):
        #
        # implementCODE
        #
        x = int()
        z = int()
        alpha = float()
        return [x, z, alpha]


    #
    # getBaseState
    #
    def OmniRobot_getBaseState(self):
        #
        # implementCODE
        #
        state = RoboCompGenericBase.TBaseState()
        return state


    #
    # resetOdometer
    #
    def OmniRobot_resetOdometer(self):
        #
        # implementCODE
        #
        pass


    #
    # setOdometer
    #
    def OmniRobot_setOdometer(self, state):
        #
        # implementCODE
        #
        pass


    #
    # setOdometerPose
    #
    def OmniRobot_setOdometerPose(self, x, z, alpha):
        #
        # implementCODE
        #
        pass


    #
    # setSpeedBase
    #
    def OmniRobot_setSpeedBase(self, advx, advz, rot):
        #
        # implementCODE
        #
        pass


    #
    # stopBase
    #
    def OmniRobot_stopBase(self):
        #
        # implementCODE
        #
        pass

    # ===================================================================
    # ===================================================================


