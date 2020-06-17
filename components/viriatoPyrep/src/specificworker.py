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
from bisect import bisect_left
from os.path import dirname, join, abspath
from pyrep import PyRep
#from pyrep.robots.mobiles.viriato import Viriato
from pyrep.robots.mobiles.viriato import Viriato
from pyrep.robots.mobiles.youbot import YouBot
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
import numpy as np
import numpy_indexed as npi
from itertools import zip_longest
import cv2
import queue

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        #self.timer.timeout.connect(self.compute)
        #self.Period = 100
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        
        SCENE_FILE = '../../etc/autonomy-lab.ttt'
        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        
        self.robot = Viriato()
        
        self.robot_centre = Dummy("Viriato_intermediate_target_base")
        self.camera_1_rgb = VisionSensor("camera_1_rgbd_sensor")
        self.camera_2_rgb = VisionSensor("camera_2_rgbd_sensor")
        self.camera_3_rgb = VisionSensor("camera_3_rgbd_sensor")
        self.camera_head = VisionSensor("real_sense_sensor")
        camera_semi_angle = np.radians(self.camera_head.get_perspective_angle())/2 
        self.cfocal = self.camera_head.get_resolution()[0]/2/np.tan(camera_semi_angle)
        self.hokuyo_base_front_left = VisionSensor("hokuyo_base_front_left")
        self.hokuyo_base_front_right = VisionSensor("hokuyo_base_front_right")
        self.hokuyo_base_back_right = VisionSensor("hokuyo_base_back_right")
        self.hokuyo_base_back_left = VisionSensor("hokuyo_base_back_left")
       
        self.joy_queue = queue.Queue(1)
        self.omnirobot_queue = queue.Queue(1)
        

    #@QtCore.Slot()
    def compute(self):
        while True:
            try:
                #start = time.time()
                self.pr.step()
                image = self.camera_head.capture_rgb()
                depth = self.camera_head.capture_depth(in_meters=True)
                # compute RGBDSimple
                h, w, d = image.shape
                list_image = image.flatten()
                self.camera_head_rgb = RoboCompCameraRGBDSimple.TImage(cameraID=0, width=w, height=h, focalx=self.cfocal, focaly=self.cfocal, alivetime=time.time(), image=list_image)
                h, w = depth.shape
                list_depth = depth.flatten()
                self.camera_head_depth = RoboCompCameraRGBDSimple.TDepth(cameraID=0, width=w, height=h, focalx=self.cfocal, focaly=self.cfocal, alivetime=time.time(), depth=list_depth)
                try:
                    self.camerargbdsimplepub_proxy.pushRGBD(self.camera_head_rgb, self.camera_head_depth)
                except Ice.Exception as e:
                    print(e)

                # compute TLaserData and publish
                ldata = self.compute_omni_laser([self.hokuyo_base_front_right,
                                                 self.hokuyo_base_front_left, 
                                                 self.hokuyo_base_back_left,
                                                 self.hokuyo_base_back_right
                                                 ],
                                                 self.robot)                   
                try:
                    self.laserpub_proxy.pushLaserData(ldata)
                except Ice.Exception as e:
                    print(e)
                
                # Move robot from data in joystick buffer
                if not self.joy_queue.empty():
                    datos = self.joy_queue.get()
                    self.update_joystick(datos)

                # Get and publish robot pose
                pose = self.robot.get_2d_pose()
                try:
                    self.bState = RoboCompGenericBase.TBaseState(x=pose[0]*1000, z=pose[1]*1000, alpha=pose[2])
                    self.omnirobotpub_proxy.pushBaseState(self.bState)
                except Ice.Exception as e:
                    print(e)

                # Move robot from data setSpeedBase
                if not self.omnirobot_queue.empty():
                    vels = self.omnirobot_queue.get()
                    self.robot.set_base_angular_velocites(vels)

                time.sleep(0.001)
                #print(time.time()-start)
            except KeyboardInterrupt:
                break

    # General laser computation
    def compute_omni_laser(self, lasers, robot):
        c_data = []
        coor = []
        for laser in lasers:
            semiwidth = laser.get_resolution()[0]/2
            semiangle = np.radians(laser.get_perspective_angle()/2)
            focal = semiwidth/np.tan(semiangle)
            data = laser.capture_depth(in_meters=True)
            m = laser.get_matrix(robot)     # these data should be read first
            imat = np.array([[m[0],m[1],m[2],m[3]],[m[4],m[5],m[6],m[7]],[m[8],m[9],m[10],m[11]],[0,0,0,1]])
            
            for i,d in enumerate(data.T):
                z = d[0]        # min if more than one row in depth image
                vec = np.array([-(i-semiwidth)*z/focal, 0, z, 1])
                res = imat.dot(vec)[:3]       # translate to robot's origin, homogeneous
                c_data.append([np.arctan2(res[0], res[1]), np.linalg.norm(res)])  # add to list in polar coordinates
        # create 360 polar rep
        c_data_np = np.asarray(c_data)
        angles = np.linspace(-np.pi, np.pi, 360)                              # create regular angular values
        positions = np.searchsorted(angles, c_data_np[:,0])               # list of closest position for each laser meas
        ldata = [RoboCompLaser.TData(a, 0) for a in angles]                   # create empty 360 angle array
        pos , medians  = npi.group_by(positions).median(c_data_np[:,1])   # group by repeated positions
        for p, m in zip_longest(pos, medians):                                # fill the angles with measures                    
            ldata[p].dist = int(m*1000)                                       # to millimeters
        return ldata
    
    def update_joystick(self, datos):
        adv = 0.0
        rot = 0.0
        side = 0.0
        for x in datos.axes:
            if x.name == "advance" and np.abs(x.value)>100:
                adv = x.value/10
            if x.name=="rotate" and np.abs(x.value)>100:
                rot = x.value/10
            if x.name=="side" and np.abs(x.value)>100:
                side = x.value/10
        self.robot.set_base_angular_velocites([adv, side, rot])

    ##################################################################################
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
        return self.camera_head_rgb, self.camera_head_depth

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
        return self.ldata, bState

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

   #self.hokuyo_base_front_left_semiangle = np.radians(self.hokuyo_base_front_left.get_perspective_angle()/2)
        #self.hokuyo_base_front_left_semiwidth = self.hokuyo_base_front_left.get_resolution()[0]/2
        #self.hokuyo_base_front_left_focal = self.hokuyo_base_front_left_semiwidth/np.tan(self.hokuyo_base_front_left_semiangle)
     
    # hokuyo_base_front_left_reading = self.hokuyo_base_front_left.capture_depth(in_meters=True)
                # hokuyo_base_front_right_reading = self.hokuyo_base_front_right.capture_depth(in_meters=True)
                # ldata = []
                # for i,d in enumerate(hokuyo_base_front_left_reading.T):
                #     angle = np.arctan2(i-(self.hokuyo_base_front_left_semiwidth), self.hokuyo_base_front_left_focal)
                #     dist = (d[0]/np.abs(np.cos(angle)))*1000
                #     ldata.append(RoboCompLaser.TData(angle-self.hokuyo_base_front_right_semiangle,dist))
                # for i,d in enumerate(hokuyo_base_front_right_reading.T):
                #     angle = np.arctan2(i-(self.hokuyo_base_front_right_semiwidth), self.hokuyo_base_front_right_focal)
                #     dist = (d[0]/np.abs(np.cos(angle)))*1000
                #     ldata.append(RoboCompLaser.TData(angle+self.hokuyo_base_front_right_semiangle,dist))
             