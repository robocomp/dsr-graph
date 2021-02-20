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
#from pyrep.robots.mobiles.viriato import Viriato
from pyrep.robots.mobiles.youbot import YouBot
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.shape import Object
from pyrep.objects.joint import Joint

import numpy as np
import numpy_indexed as npi
from itertools import zip_longest
import cv2
import queue

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        
        SCENE_FILE = '../../etc/pioneer.ttt'

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()

        self.robot_object = Shape("Pioneer")
        self.back_left_wheel = Joint("p3at_back_left_wheel_joint")
        self.back_right_wheel = Joint("p3at_back_right_wheel_joint")
        self.front_left_wheel = Joint("p3at_front_left_wheel_joint")
        self.front_right_wheel = Joint("p3at_front_right_wheel_joint")
        self.radius = 110 # mm
        self.semi_width = 140 # mm

        # Each laser is composed of two cameras. They are converted into a 360 virtual laser
        #self.hokuyo_base_front_right = VisionSensor("SICK_TiM310_sensor1")
        #self.hokuyo_base_front_left = VisionSensor("SICK_TiM310_sensor2")

        self.ldata = []
        self.joystick_newdata = []
        self.speed_robot = []
        self.speed_robot_ant = []
        self.last_received_data_time = 0

    def compute(self):
        cont = 0
        start = time.time()
        while True:
            self.pr.step()
            #self.read_laser()
            self.read_joystick()
            self.read_robot_pose()
            self.move_robot()

            elapsed = time.time()-start
            if elapsed < 0.05:
                time.sleep(0.05-elapsed)
            cont += 1
            if time.time()-start > 1:
                print("Freq -> ", cont)
                cont = 0
                start = time.time()

    ###########################################
    ### LASER get and publish laser data
    ###########################################
    def read_laser(self):
        self.ldata = self.compute_omni_laser([self.hokuyo_base_front_right,
                                              self.hokuyo_base_front_left], self.robot_object)
        try:
            self.laserpub_proxy.pushLaserData(self.ldata)
        except Ice.Exception as e:
            print(e)

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata: #and (time.time() - self.joystick_newdata[1]) > 0.1:
            datos = self.joystick_newdata[0]
            adv = 0.0
            rot = 0.0
            for x in datos.axes:
                if x.name == "advance":
                    adv = x.value if np.abs(x.value) > 1 else 0
                if x.name == "rotate":
                    rot = x.value if np.abs(x.value) > 0.01 else 0

            converted = self.convert_base_speed_to_motors_speed(adv, rot)
            print("Joystick ", [adv, rot], converted)
            self.joystick_newdata = None
            self.last_received_data_time = time.time()
        else:
            elapsed = time.time() - self.last_received_data_time
            if elapsed > 2 and elapsed < 3:
                self.convert_base_speed_to_motors_speed(0,0)

    def convert_base_speed_to_motors_speed(self, adv, rot):
        #  adv = r*(Wl + Wr)/2
        #  rot = r*(-Wl + Wr)/2c
        #  isolating Wl,Wr
        #  Wl = ( adv - c*rot ) / r
        #  Wr = ( adv + c*rot ) / r
        left_vel = (adv + self.semi_width * rot) / self.radius
        right_vel = (adv - self.semi_width * rot) / self.radius
        self.back_left_wheel.set_joint_target_velocity(left_vel)
        self.back_right_wheel.set_joint_target_velocity(right_vel)
        self.front_left_wheel.set_joint_target_velocity(left_vel)
        self.front_right_wheel.set_joint_target_velocity(right_vel)
    ###########################################
    ### ROBOT POSE get and publish robot position
    ###########################################
    def read_robot_pose(self):
        #pose = self.robot.get_2d_pose()
        pose = self.robot_object.get_pose()
        linear_vel, ang_vel = self.robot_object.get_velocity()
        # print("Veld:", linear_vel, ang_vel)
        try:
            isMoving = np.abs(linear_vel[0]) > 0.01 or np.abs(linear_vel[1]) > 0.01 or np.abs(ang_vel[2]) > 0.01
            self.bState = RoboCompGenericBase.TBaseState(x=pose[0] * 1000,
                                                         z=pose[1] * 1000,
                                                         alpha=pose[2],
                                                         advVx=linear_vel[0] * 1000,
                                                         advVz=linear_vel[1] * 1000,
                                                         rotV=ang_vel[2],
                                                         isMoving=isMoving)
            self.omnirobotpub_proxy.pushBaseState(self.bState)
        except Ice.Exception as e:
            print(e)

    ###########################################
    ### MOVE ROBOT from Omnirobot interface
    ###########################################
    def move_robot(self):

        if self.speed_robot != self.speed_robot_ant:  # or (isMoving and self.speed_robot == [0,0,0]):
            self.convert_base_speed_to_motors_speed(self.speed_robot)
        #print("Velocities sent to robot:", self.speed_robot)
            self.speed_robot_ant = self.speed_robot

    ########################################
    ## General laser computation
    ########################################
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
                #print(np.arctan2(res[0], res[1]))

        # create N degrees polar rep
        N = 360
        min_rad = -np.pi
        max_rad = np.pi
        c_data_np = np.asarray(c_data)
        angles = np.linspace(min_rad, max_rad, N)                         # create regular angular values
        positions = np.searchsorted(angles, c_data_np[:, 0])               # list of closest position for each laser meas
        ldata = [RoboCompLaser.TData(a, 0) for a in angles]               # create empty 360 angle array
        pos, medians = npi.group_by(positions).median(c_data_np[:, 1])   # group by repeated positions
        for p, m in zip_longest(pos, medians):                            # fill the angles with measures
            ldata[p].dist = int(m*1000)   # to millimeters
        if ldata[0] == 0:
            ldata[0] = self.semi_width       #half robot width
        for i in range(0, len(ldata)):
            if ldata[i].dist == 0:
                ldata[i].dist = ldata[i-1].dist
        #ldata[0].dist = ldata[len(data)-1].dist

        return ldata

    ##################################################################################
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    ###################################################################################
    def JoystickAdapter_sendData(self, data):
        self.joystick_newdata = [data, time.time()]


    ##################################################################################
    #                       Methods for CameraRGBDSimple
    # ===============================================================================
    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self, camera):
        return RoboCompCameraRGBDSimple.TRGBD(self.cameras[camera]["rgb"], self.cameras[camera]["depth"])

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self, camera):
        return self.cameras[camera]["depth"]
    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self, camera):
        return self.cameras[camera]["rgb"]

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
        self.speed_robot = self.convert_base_speed_to_radians(advz, advx, rot)

    #
    # stopBase
    #
    def OmniRobot_stopBase(self):
        pass

    # ===================================================================
    # CoppeliaUtils
    # ===================================================================
    def CoppeliaUtils_addOrModifyDummy(self, type, name, pose):
        if not Dummy.exists(name):
            dummy = Dummy.create(0.1)
            # one color for each type of dummy
            if type==RoboCompCoppeliaUtils.TargetTypes.Info:
                pass
            if type == RoboCompCoppeliaUtils.TargetTypes.Hand:
                pass
            if type == RoboCompCoppeliaUtils.TargetTypes.HeadCamera:
                pass
            dummy.set_name(name)
        else:
            dummy = Dummy(name)
            parent_frame_object = None
            if type == RoboCompCoppeliaUtils.TargetTypes.HeadCamera:
                parent_frame_object = Dummy("viriato_head_camera_pan_tilt")
            #print("Coppelia ", name, pose.x/1000, pose.y/1000, pose.z/1000)
            dummy.set_position([pose.x / 1000., pose.y / 1000., pose.z / 1000.], parent_frame_object)
            dummy.set_orientation([pose.rx, pose.ry, pose.rz], parent_frame_object)

