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
import os, time, queue
from os.path import dirname, join, abspath
from pyrep import PyRep
#from pyrep.robots.mobiles.viriato import Viriato
from pyrep.robots.mobiles.viriato import Viriato
from pyrep.objects.vision_sensor import VisionSensor
import numpy as np
import cv2

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        #self.timer.timeout.connect(self.compute)
        #self.Period = 100
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        
        SCENE_FILE = '/home/robocomp/software/vrep-simulator/viriato/alab-completo-nolaser-pyrep.ttt'
        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        
        self.agent = Viriato()
        self.camera = VisionSensor("real_sense_sensor")
        camera_semi_angle = np.radians(self.camera.get_perspective_angle())/2 
        self.cfocal = self.camera.get_resolution()[0]/2/np.tan(camera_semi_angle)
        self.laser = VisionSensor("laser")
        laser_angle = np.radians(self.laser.get_perspective_angle())
        self.lfocal = self.laser.get_resolution()[0]/2/np.tan(laser_angle/2)

        self.joy_queue = queue.Queue(1)
        self.omnirobot_queue = queue.Queue(1)

    #@QtCore.Slot()
    def compute(self):
        
        while True:
            try:
                #start = time.time()
                self.pr.step()
                self.image = self.camera.capture_rgb()
                self.depth = self.camera.capture_depth(in_meters=True)
                laser_reading = self.laser.capture_depth(in_meters=True)
                # compute RGBDSimple
                h, w, d = self.image.shape
                self.timg = RoboCompCameraRGBDSimple.TImage(cameraID=0, width=w, height=h, focalx=self.cfocal, focaly=self.cfocal, alivetime=time.time(), image=np.copy(self.image))
                h, w = self.depth.shape
                self.tdepth = RoboCompCameraRGBDSimple.TDepth(cameraID=0, width=w, height=h, focalx=self.lfocal, focaly=self.lfocal, alivetime=time.time(), depth=np.copy(self.depth))
                try:
                    self.camerargbdsimplepub_proxy.pushRGBD(self.timg, self.tdepth)
                    
                except Ice.Exception as e:
                    print(e)

                # compute TLaserData and publish
                laser_width = laser_reading.shape[1]
                ldata = []
                for i,d in enumerate(laser_reading.T):
                    angle = np.arctan2(i-(laser_width/2), self.lfocal)
                    dist = (d[0]/np.abs(np.cos(angle)))*1000
                    ldata.append(RoboCompLaser.TData(angle,dist))
                    
                try:
                    self.laserpub_proxy.pushLaserData(ldata)
                except Ice.Exception as e:
                    print(e)
                
                # Move robot from data in joystick buffer
                if not self.joy_queue.empty():
                    datos = self.joy_queue.get()
                    adv = 0.
                    rot = 0.
                    side = 0.
                    for x in datos.axes:
                        if x.name == "advance" and np.abs(x.value)>0.25:
                            adv = x.value
                        if x.name=="rotate" and np.abs(x.value)>0.25:
                            rot = x.value
                        if x.name=="side" and np.abs(x.value)>0.25:
                            side = x.value
                    self.agent.set_base_angular_velocites([adv, side, rot])

                # Get and publish robot pose
                pose = self.agent.get_2d_pose()
                try:
                    self.bState = RoboCompGenericBase.TBaseState(x=pose[0]*1000, z=pose[1]*1000, alpha=pose[2])
                    self.omnirobotpub_proxy.pushBaseState(self.bState)
                except Ice.Exception as e:
                    print(e)

                # Move robot from data setSpeedBase
                if not self.omnirobot_queue.empty():
                    vels = self.omnirobot_queue.get()
                    self.agent.set_base_angular_velocites(vels)

                time.sleep(0.001)
                #print(time.time()-start)
            except KeyboardInterrupt:
                break

        
    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):
        self.joy_queue.put(data)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self):
        return [self.timg, self.tdeph]

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self):
        return self.tdepth
    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self):
        return self.timg

    #######################################################
    #### Laser
    #######################################################

    #
    # getLaserAndBStateData
    #
    def Laser_getLaserAndBStateData(self):
        bState = RoboCompGenericBase.TBaseState()
        return [self.ldata, bState]

    #
    # getLaserConfData
    #
    def Laser_getLaserConfData(self):
        ret = RoboCompLaser.LaserConfData()
        return ret

    #
    # getLaserData
    #
    def Laser_getLaserData(self):
        return self.ldata

    ##############################################
    ## Omnibase
    #############################################

    #
    # correctOdometer
    #
    def OmniRobot_correctOdometer(self, x, z, alpha):
        pass

    #
    # getBasePose
    #
    def OmniRobot_getBasePose(self):
        #
        # implementCODE
        #
        x = self.bState.x
        z = self.bState.z
        alpha = self.bState.alpha
        return [x, z, alpha]

    #
    # getBaseState
    #
    def OmniRobot_getBaseState(self):
        return self.bState

    #
    # resetOdometer
    #
    def OmniRobot_resetOdometer(self):
        pass

    #
    # setOdometer
    #
    def OmniRobot_setOdometer(self, state):
        pass

    #
    # setOdometerPose
    #
    def OmniRobot_setOdometerPose(self, x, z, alpha):
        pass

    #
    # setSpeedBase
    #
    def OmniRobot_setSpeedBase(self, advx, advz, rot):
        self.omnirobot_queue.put([advz, advx, rot])

    #
    # stopBase
    #
    def OmniRobot_stopBase(self):
        pass

    # ===================================================================
    # ===================================================================


