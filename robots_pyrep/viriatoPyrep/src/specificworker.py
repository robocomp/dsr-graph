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
import sys

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
#from pyrep.objects.shape import Object
from pyrep.objects.joint import Joint
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
import numpy as np
import numpy_indexed as npi
from itertools import zip_longest
import cv2
from threading import Lock
from math import *

class TimeControl:
    def __init__(self, period_):
        self.counter = 0
        self.start = time.time()  # it doesn't exist yet, so initialize it
        self.start_print = time.time()  # it doesn't exist yet, so initialize it
        self.period = period_

    def wait(self):
        elapsed = time.time() - self.start
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        self.start = time.time()
        self.counter += 1
        if time.time() - self.start_print > 1:
            print("Freq -> ", self.counter)
            self.counter = 0
            self.start_print = time.time()

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        
        #SCENE_FILE = '../../etc/autonomy_lab_no_arm_bill.ttt'
        #SCENE_FILE = '../../etc/autonomy_lab_bill.ttt'
        SCENE_FILE = '../../etc/autonomy_lab_no_arm_no_middle_wall.ttt'

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        
        #self.robot = Viriato()
        self.robot = YouBot()
        self.robot_object = Shape("youBot")
        #self.robot_left_arm = Shape("viriato_left_arm")
        #self.robot_left_arm_tip = Dummy("viriato_left_arm_tip")
        # self.ViriatoBase_WheelRadius = 76.2  #mm real robot
        self.ViriatoBase_WheelRadius = 44  # mm coppelia
        self.ViriatoBase_DistAxes = 380.
        self.ViriatoBase_AxesLength = 422.
        self.ViriatoBase_Rotation_Factor = 8.1  # it should be (DistAxes + AxesLength) / 2
        self.bState_ant = RoboCompGenericBase.TBaseState()

        self.cameras = {}
        # cam = VisionSensor("wall_camera_1")
        # self.cameras["wall_camera_1"] = { "handle": cam,
        #                                             "id": 1,
        #                                             "angle": np.radians(cam.get_perspective_angle()),
        #                                             "width": cam.get_resolution()[0],
        #                                             "height": cam.get_resolution()[1],
        #                                             "depth": 3,
        #                                             "focal": cam.get_resolution()[0]/np.tan(np.radians(cam.get_perspective_angle())),
        #                                             "rgb": np.array(0),
        #                                             "depth": np.ndarray(0) }
        # cam = VisionSensor("camera_2_rgbd_sensor")                                            
        # self.cameras["camera_2_rgbd_sensor"] = {    "handle": cam, 
        #                                             "id": 2,
        #                                             "angle": np.radians(cam.get_perspective_angle()), 
        #                                             "width": cam.get_resolution()[0],
        #                                             "height": cam.get_resolution()[1],
        #                                             "focal": cam.get_resolution()[0]/np.tan(np.radians(cam.get_perspective_angle())), 
        #                                             "rgb": np.array(0), 
        #                                             "depth": np.ndarray(0) }
        # cam = VisionSensor("camera_3_rgbd_sensor")                                            
        # self.cameras["camera_3_rgbd_sensor"] = {    "handle": cam, 
        #                                             "id": 3,
        #                                             "angle": np.radians(cam.get_perspective_angle()), 
        #                                             "width": cam.get_resolution()[0],
        #                                             "height": cam.get_resolution()[1],
        #                                             "focal": cam.get_resolution()[0]/np.tan(np.radians(cam.get_perspective_angle())), 
        #                                             "rgb": np.array(0), 
        #                                             "depth": np.ndarray(0) }
        #
        # robot head camera
        self.initialize_cameras()
        
        # camera tilt motor
        self.viriato_head_camera_pan_joint_name = "viriato_head_camera_pan_joint"
        self.viriato_head_camera_tilt_joint_name = "viriato_head_camera_tilt_joint"
        self.viriato_head_camera_tilt_joint = Joint(self.viriato_head_camera_tilt_joint_name)
        self.viriato_head_camera_pan_joint = Joint(self.viriato_head_camera_pan_joint_name)

        # Each laser is composed of two cameras. They are converted into a 360 virtual laser
        self.hokuyo_base_front_left = VisionSensor("hokuyo_base_front_left")
        self.hokuyo_base_front_right = VisionSensor("hokuyo_base_front_right")
        self.hokuyo_base_back_right = VisionSensor("hokuyo_base_back_right")
        self.hokuyo_base_back_left = VisionSensor("hokuyo_base_back_left")

        # Read existing people
        self.people = {}
        if Dummy.exists("Bill"):
            self.people["Bill"] = Dummy("Bill")
        for i in range(0,2):
            name = "Bill#" + str(i)
            if Dummy.exists(name):
                self.people[name] = Dummy(name)

        self.joystick_newdata = []
        self.speed_robot = []
        self.speed_robot_ant = []
        self.last_received_data_time = 0

        # PoseEstimation
        self.tm = TransformManager()
        self.tm.add_transform("origin", "world",
                              pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0.0, 0.0, 0.0]),
                                                  [0.0, 0.0, 0.0])
                              )
        self.robot_full_pose_write = RoboCompFullPoseEstimation.FullPoseEuler()
        self.robot_full_pose_read = RoboCompFullPoseEstimation.FullPoseEuler()
        self.mutex_pose = Lock()
        self.primera_vez = True
        self.new_coppelia_script_call_available = False

    def initialize_cameras(self):
        print("Initialize camera")
        self.cameras.clear()
        self.cameras.clear()
        cam = VisionSensor("viriato_head_camera_sensor")
        self.cameras["viriato_head_camera_sensor"] = {    "handle": cam,
                                                                "id": 0,
                                                                "angle": np.radians(cam.get_perspective_angle()),
                                                                "width": cam.get_resolution()[0],
                                                                "height": cam.get_resolution()[1],
                                                                "focal": (cam.get_resolution()[0]/2) / np.tan(np.radians(cam.get_perspective_angle()/2.0)),
                                                                "rgb": np.array(0),
                                                                "depth": np.ndarray(0) }

        # cam = VisionSensor("wall_camera_1")
        # self.cameras["wall_camera_1"] = {"handle": cam,
        #                                             "id": 1,
        #                                             "angle": np.radians(cam.get_perspective_angle()),
        #                                             "width": cam.get_resolution()[0],
        #                                             "height": cam.get_resolution()[1],
        #                                             "depth": 3,
        #                                             "focal": cam.get_resolution()[0]/2 / np.tan(np.radians(cam.get_perspective_angle()/2.0)),
        #                                             "rgb": np.array(0),
        #                                             "depth": np.ndarray(0)
#                                 }
    #@QtCore.Slot()
    def compute(self):
        tc = TimeControl(0.05)  # 50 millis -> 20Hz
        while True:
            self.pr.step()
            self.read_cameras()
            self.read_people()
            self.read_laser()
            self.read_joystick()
            self.read_robot_speed()
            self.read_robot_pose()
            #self.read_robot_arm_tip()
            self.read_pan_tilt()

            if self.new_coppelia_script_call_available:
                print("Coppelia dummy script sent ", self.coppelia_nose_speed)
                self.pr.script_call("set_velocity@viriato_head_pan_tilt_nose_target", 1, (), self.coppelia_nose_speed, (), '')
                self.new_coppelia_script_call_available = False

            # if self.primera_vez:
            #     self.pr.script_call("set_velocity@viriato_head_pan_tilt_nose_target", 1, (),
            #                         (-0.00064, -0.00024, 0.0), (), '')
            #     self.primera_vez = False

            tc.wait()

    ###########################################
    ### CAMERAS get and publish cameras data
    ###########################################
    def read_cameras(self):
        for name, cam in self.cameras.items():
            # check resolution change
            if cam["width"] != cam["handle"].get_resolution()[0] or cam["height"] != cam["handle"].get_resolution()[1]:
                print("Resolution changed")
                self.initialize_cameras()
                break

            image_float = cam["handle"].capture_rgb()
#            print("len", len(image_float))
            depth = cam["handle"].capture_depth(True)
            image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                  dtype=cv2.CV_8U)
            cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"], width=cam["width"], height=cam["height"],
                                                         depth=3, focalx=cam["focal"], focaly=cam["focal"],
                                                         alivetime=time.time(), image=image.tobytes())
            cam["depth"] = RoboCompCameraRGBDSimple.TDepth(cameraID=cam["id"], width=cam["handle"].get_resolution()[0], height=cam["handle"].get_resolution()[1],
                                                           focalx=cam["focal"], focaly=cam["focal"],
                                                           alivetime=time.time(), depthFactor=1.0, depth=depth.tobytes())

            try:
                self.camerargbdsimplepub_proxy.pushRGBD(cam["rgb"], cam["depth"])
            except Ice.Exception as e:
                print(e)

    ###########################################
    ### PEOPLE get and publish people position
    ###########################################
    def read_people(self):
        people_data = RoboCompHumanToDSRPub.PeopleData()
        people_data.timestamp = time.time()
        people = []  # RoboCompHumanToDSRPub.People()
        for name, handle in self.people.items():
            pos = handle.get_position()
            rot = handle.get_orientation()
            person = RoboCompHumanToDSRPub.Person(len(people), pos[0] * 1000, pos[1] * 1000, pos[2] * 1000,
                                                  pi - rot[2] - pi/2,
                                                  {})
            people.append(person)
        try:
            people_data.peoplelist = people
            self.humantodsrpub_proxy.newPeopleData(people_data)
        except Ice.Exception as e:
            print(e)

    ###########################################
    ### LASER get and publish laser data
    ###########################################
    def read_laser(self):
        ldata = self.compute_omni_laser([self.hokuyo_base_front_right,
                                         self.hokuyo_base_front_left,
                                         self.hokuyo_base_back_left,
                                         self.hokuyo_base_back_right
                                         ], self.robot)
        try:
            self.laserpub_proxy.pushLaserData(ldata)
        except Ice.Exception as e:
            print(e)

    ###########################################
    ###  read and move the robot
    ###########################################
    def read_robot_speed(self):
        if self.speed_robot:  # and (time.time() - self.joystick_newdata[1]) > 0.1:
            self.robot.set_base_angular_velocites(self.speed_robot)
            self.speed_robot = None

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata: #and (time.time() - self.joystick_newdata[1]) > 0.1:
            adv = 0.0
            rot = 0.0
            side = 0.0
            pan = 0.0
            tilt = 0.0
            head_moves = False

            for x in self.joystick_newdata[0].axes:
                if x.name == "advance":
                    adv = x.value if np.abs(x.value) > 1 else 0  # mm/sg
                if x.name == "rotate":
                    rot = x.value if np.abs(x.value) > 0.01 else 0  # rads/sg
                if x.name == "side":
                    side = x.value if np.abs(x.value) > 1 else 0
                if x.name == "pan":
                    pan = x.value if np.abs(x.value) > 0.01 else 0
                    head_moves = True
                if x.name == "tilt":
                    tilt = x.value if np.abs(x.value) > 0.01 else 0
                    head_moves = True

            # print("Joystick ", adv, rot, side)
            converted = self.convert_base_speed_to_radians(adv, side, rot)
            # self.robot.set_base_angular_velocites([adv, side, rot])
            self.robot.set_base_angular_velocites(converted)
            #
            if (head_moves):
                dummy = Dummy("viriato_head_pan_tilt_nose_target")
                pantilt = Dummy("viriato_head_camera_pan_tilt")
                pose = dummy.get_position(pantilt)
                dummy.set_position([pose[0], pose[1] - pan / 10, pose[2] + tilt / 10], pantilt)

            self.joystick_newdata = None
            #self.last_received_data_time = time.time()
        else:
            elapsed = time.time() - self.last_received_data_time
            if elapsed > 2 and elapsed < 3:
                self.robot.set_base_angular_velocites([0, 0, 0])

    ###########################################
    ### ROBOT POSE get and publish robot position
    ###########################################
    def read_robot_pose(self):
        pose = self.robot.get_2d_pose()
        linear_vel, ang_vel = self.robot_object.get_velocity()
        # print("Veld:", linear_vel, ang_vel)

        isMoving = np.abs(linear_vel[0]) > 0.01 or np.abs(linear_vel[1]) > 0.01 or np.abs(ang_vel[2]) > 0.01
        self.bState = RoboCompGenericBase.TBaseState(x=pose[0] * 1000,
                                                     z=pose[1] * 1000,
                                                     alpha=pose[2],
                                                     advVx=linear_vel[0] * 1000,
                                                     advVz=linear_vel[1] * 1000,
                                                     rotV=ang_vel[2],
                                                     isMoving=isMoving)

        try:        # parameters are adjusted for Coppelia sensibility
            if not np.allclose([self.bState.x, self.bState.z], [self.bState_ant.x, self.bState_ant.z], rtol=1e-03, atol=1) \
            or not np.isclose(self.bState.alpha, self.bState_ant.alpha, rtol=1e-03, atol=1e-02):
                self.omnirobotpub_proxy.pushBaseState(self.bState)
                self.bState_ant = self.bState
                print("now")

        except Ice.Exception as e:
            print(e)

        # self.tm.add_transform("world", "robot", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #                                                             ([0.0, 0.0, pose[2] - np.pi]),
        #                                                             [pose[0] * 1000.0, pose[1] * 1000.0, 0.0]
        #                                                             ))
        #
        # t = self.tm.get_transform("origin", "robot")
        # angles = pyrot.extrinsic_euler_xyz_from_active_matrix(t[0:3, 0:3])
        #
        # self.robot_full_pose_write.x = t[0][3]
        # self.robot_full_pose_write.y = t[1][3]
        # self.robot_full_pose_write.z = t[2][3]
        # self.robot_full_pose_write.rx = angles[0]
        # self.robot_full_pose_write.ry = angles[1]
        # self.robot_full_pose_write.rz = angles[2]
        # self.robot_full_pose_write.vx = linear_vel[0] * 1000.0
        # self.robot_full_pose_write.vy = linear_vel[1] * 1000.0
        # self.robot_full_pose_write.vz = linear_vel[2] * 1000.0
        # self.robot_full_pose_write.vrx = ang_vel[0]
        # self.robot_full_pose_write.vry = ang_vel[1]
        # self.robot_full_pose_write.vrz = ang_vel[2]
        #
        # # swap
        # self.mutex_pose.acquire()
        # self.robot_full_pose_write, self.robot_full_pose_read = self.robot_full_pose_read, self.robot_full_pose_write
        # self.mutex_pose.release()

    ###########################################
    ### ROBOT POSE get and publish robot position
    ###########################################
    def read_robot_arm_tip(self):
        trans = self.robot_left_arm_tip.get_position(self.robot_object)
        rot = self.robot_left_arm_tip.get_orientation(self.robot_object)
        qt = self.robot_left_arm_tip.get_quaternion(self.robot_object)
        linear_vel, ang_vel = self.robot_left_arm_tip.get_velocity()
        #print(trans, rot, linear_vel)
        try:
            isMoving = np.sum(np.abs(linear_vel)) > 0.005 or np.sum(np.abs(ang_vel)) > 0.005
            if isMoving:
                self.arm_state = RoboCompKinovaArmPub.TArmState(x=trans[0]*1000, y=trans[1]*1000, z=trans[2]*1000,
                                                            rx=rot[0], ry=rot[1], rz=rot[2],
                                                            qta=qt[0], qtb=qt[1], qtc=qt[2], qtd=qt[3])
                self.kinovaarmpub_proxy.newArmState(self.arm_state)
        except Ice.Exception as e:
            print(e)

    ###########################################
    ### Viriato head camera tilt motor. Command from JointMotor interface
    ############################################
    def read_pan_tilt(self):
        mpan = RoboCompJointMotor.MotorState()
        mpan.pos = self.viriato_head_camera_pan_joint.get_joint_position()  # rads
        mtilt = RoboCompJointMotor.MotorState()
        mtilt.pos = self.viriato_head_camera_tilt_joint.get_joint_position()
        motors = dict({self.viriato_head_camera_pan_joint_name: mpan,
                       self.viriato_head_camera_tilt_joint_name: mtilt})
        try:
            #print("Joints: ", motors)
            self.jointmotorpub_proxy.motorStates(motors)
        except Ice.Exception as e:
            print(e)

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
            # print("m", m)
            # print("m.shape", m.shape)
            if type(m) != list:
                m = [item for sublist in m for item in sublist]
            imat = np.array([[m[0],m[1],m[2],m[3]],[m[4],m[5],m[6],m[7]],[m[8],m[9],m[10],m[11]],[0,0,0,1]])

            for i,d in enumerate(data.T):
                z = d[0]        # min if more than one row in depth image
                vec = np.array([-(i-semiwidth)*z/focal, 0, z, 1])
                res = imat.dot(vec)[:3]       # translate to robot's origin, homogeneous
                c_data.append([np.arctan2(res[0], res[1]), np.linalg.norm(res)])  # add to list in polar coordinates

        # create 360 polar rep
        c_data_np = np.asarray(c_data)
        angles = np.linspace(-np.pi, np.pi, 360)                          # create regular angular values
        positions = np.searchsorted(angles, c_data_np[:,0])               # list of closest position for each laser meas
        ldata = [RoboCompLaser.TData(a, 0) for a in angles]               # create empty 360 angle array
        pos , medians  = npi.group_by(positions).median(c_data_np[:,1])   # group by repeated positions
        for p, m in zip_longest(pos, medians):                            # fill the angles with measures
            ldata[p].dist = int(m*1000)   # to millimeters
        if ldata[0] == 0:
            ldata[0] = 200       #half robot width
        for i in range(1, len(ldata)):
            if ldata[i].dist == 0:
                ldata[i].dist = ldata[i-1].dist

        return ldata

    def convert_base_speed_to_radians(self, adv, side, rot):
        # rot has to be neg so neg rot speeds go clock wise. It is probably a sign in Pyrep forward kinematics
        return [adv / self.ViriatoBase_WheelRadius, side / self.ViriatoBase_WheelRadius, rot * self.ViriatoBase_Rotation_Factor]

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
        #converted = self.convert_base_speed_to_radians(advz, advx, rot)
        self.speed_robot = self.convert_base_speed_to_radians(advz, advx, rot)
        #self.robot.set_base_angular_velocites(converted)

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
            dummy.set_name(name)
        else:
            dummy = Dummy(name)
            parent_frame_object = None
            if type == RoboCompCoppeliaUtils.TargetTypes.HeadCamera:
                parent_frame_object = Dummy("viriato_head_camera_pan_tilt")          # target in parent's reference system

            # We CHANGE here axis to comply with Coppelia configuration for pan-tilt axis: x -> y; y -> x; z -> -z
            #print("Coppelia sent", name, pose.y/1000, pose.z/1000, -pose.z/1000)
            dummy.set_position([pose.y / 1000., pose.x / 1000., -pose.z / 1000.], parent_frame_object)
            dummy.set_orientation([pose.rx, pose.ry, pose.rz], parent_frame_object)

    def CoppeliaUtils_setDummySpeed(self, type, name, speed):
        if not Dummy.exists(name):
            print("Warning. Attempt to set speed to a non existent dummy", name)
            return
        else:
            # We CHANGE here axis to comply with Coppelia configuration for pan-tilt axis: x -> y; y -> x; z -> -z

            self.new_coppelia_script_call_available = True
            self.coppelia_nose_speed =  (speed.vy/1000, speed.vx/1000, -speed.vz/1000)

    def FullPoseEstimation_getFullPoseEuler(self):
        self.mutex_pose.acquire()
        try:
            return self.robot_full_pose_read
        finally:
            self.mutex_pose.release()


    def FullPoseEstimation_getFullPoseMatrix(self):
        t = self.tm.get_transform("origin", "robot")
        m = RoboCompFullPoseEstimation.FullPoseMatrix()
        m.m00 = t[0][0]
        m.m01 = t[0][1]
        m.m02 = t[0][2]
        m.m03 = t[0][3]
        m.m10 = t[1][0]
        m.m11 = t[1][1]
        m.m12 = t[1][2]
        m.m13 = t[1][3]
        m.m20 = t[2][0]
        m.m21 = t[2][1]
        m.m22 = t[2][2]
        m.m23 = t[2][3]
        m.m30 = t[3][0]
        m.m31 = t[3][1]
        m.m32 = t[3][2]
        m.m33 = t[3][3]
        return m

    def FullPoseEstimation_setInitialPose(self, x, y, z, rx, ry, rz):
        # should move robot in Coppelia to designated pose
        self.tm.add_transform("origin", "world",
                              pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([rx, ry, rz]), [x, y, z])
                              )

    ######################################################################
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
             
