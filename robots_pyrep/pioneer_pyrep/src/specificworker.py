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
import time
from pyrep import PyRep
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
import numpy as np
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
import cv2
from threading import Lock
import itertools as it
import numpy_indexed as npi


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
            print("Freq -> ", self.counter, " Hz")
            self.counter = 0
            self.start_print = time.time()


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.tm = TransformManager()
        self.tm.add_transform("origin", "world",
                              pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0.0, 0.0, 0.0]), [0.0, 0.0, 0.0])
                              )

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        #SCENE_FILE = '../../etc/informatica.ttt'
        SCENE_FILE = '/home/robocomp/robocomp/components/robocomp-pioneer/etc/exterior-robolab.ttt'

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()

        self.robot_object = Shape("Pioneer")
        self.back_left_wheel = Joint("p3at_back_left_wheel_joint")
        self.back_right_wheel = Joint("p3at_back_right_wheel_joint")
        self.front_left_wheel = Joint("p3at_front_left_wheel_joint")
        self.front_right_wheel = Joint("p3at_front_right_wheel_joint")
        self.radius = 110  # mm
        self.semi_width = 140  # mm

        # cameras
        self.cameras_write = {}
        self.cameras_read = {}

        self.front_left_camera_name = "pioneer_camera_left"
        cam = VisionSensor(self.front_left_camera_name)
        self.cameras_write[self.front_left_camera_name] = {"handle": cam,
                                                     "id": 0,
                                                     "angle": np.radians(cam.get_perspective_angle()),
                                                     "width": cam.get_resolution()[0],
                                                     "height": cam.get_resolution()[1],
                                                     "focal": (cam.get_resolution()[0] / 2) / np.tan(
                                                         np.radians(cam.get_perspective_angle() / 2)),
                                                     "rgb": np.array(0),
                                                     "depth": np.ndarray(0)}

        self.front_right_camera_name = "pioneer_camera_right"
        cam = VisionSensor(self.front_right_camera_name)
        self.cameras_write[self.front_right_camera_name] = {"handle": cam,
                                                     "id": 1,
                                                     "angle": np.radians(cam.get_perspective_angle()),
                                                     "width": cam.get_resolution()[0],
                                                     "height": cam.get_resolution()[1],
                                                     "focal": (cam.get_resolution()[0] / 2) / np.tan(
                                                         np.radians(cam.get_perspective_angle() / 2)),
                                                     "rgb": np.array(0),
                                                     "depth": np.ndarray(0)}


        self.cameras_read = self.cameras_write.copy()
        self.mutex_c = Lock()

        # laser
        self.ldata_write = []
        self.ldata_read = []

        # PoseEstimation
        self.robot_full_pose_write = RoboCompFullPoseEstimation.FullPoseEuler()
        self.robot_full_pose_read = RoboCompFullPoseEstimation.FullPoseEuler()
        self.mutex = Lock()


        self.ldata = []
        self.joystick_newdata = []
        self.speed_robot = []
        self.speed_robot_ant = []
        self.last_received_data_time = 0

    def compute(self):
        tc = TimeControl(0.05)
        while True:
            self.pr.step()
            self.read_laser()
            self.read_cameras([self.front_left_camera_name, self.front_right_camera_name])
            self.read_joystick()
            self.read_robot_pose()
            self.move_robot()

            tc.wait()

    ###########################################
    ### LASER get and publish laser data
    ###########################################

    def read_laser(self):
        data = self.pr.script_call("get_depth_data@Hokuyo_front", 1)
        if len(data[1]) > 0:
            polar = np.zeros(shape=(int(len(data[1]) / 3), 2))
            i = 0
            for x, y, z in self.grouper(data[1], 3):  # extract non-intersecting groups of 3
                polar[i] = [-np.arctan2(y, x), np.linalg.norm([x, y])]  # add to list in polar coordinates
                i += 1

            angles = np.linspace(-np.radians(120), np.radians(120), 360)  # create regular angular values
            positions = np.searchsorted(angles,
                                        polar[:, 0])  # list of closest position in polar for each laser measurement
            self.ldata_write = [RoboCompLaser.TData(a, 0) for a in angles]  # create empty 360 angle array
            pos, medians = npi.group_by(positions).median(polar[:, 1])  # group by repeated positions
            for p, m in it.zip_longest(pos, medians):  # fill the angles with measures
                if p < len(self.ldata_write):
                    self.ldata_write[p].dist = int(m * 1000)  # to millimeters
            if self.ldata_write[0] == 0:
                self.ldata_write[0] = 200  # half robot width
            for i in range(0, len(self.ldata_write)):
                if self.ldata_write[i].dist == 0:
                    self.ldata_write[i].dist = self.ldata_write[i - 1].dist

            self.ldata_read, self.ldata_write = self.ldata_write, self.ldata_read

            try:
                self.laserpub_proxy.pushLaserData(self.ldata_read)
            except Ice.Exception as e:
                print(e)

    def grouper(self, inputs, n, fillvalue=None):
        iters = [iter(inputs)] * n
        return it.zip_longest(*iters, fillvalue=fillvalue)

    ###########################################
    ### CAMERAS get and publish cameras data
    ###########################################
    def read_cameras(self, camera_names):
        for camera_name in camera_names:
            cam = self.cameras_write[camera_name]
            image_float = cam["handle"].capture_rgb()
            depth = cam["handle"].capture_depth(True)
            image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                  dtype=cv2.CV_8U)
            #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"], width=cam["width"], height=cam["height"],
                                                         depth=3, focalx=cam["focal"], focaly=cam["focal"],
                                                         alivetime=time.time(), image=image.tobytes())
            cam["depth"] = RoboCompCameraRGBDSimple.TDepth(cameraID=cam["id"], width=cam["handle"].get_resolution()[0],
                                                           height=cam["handle"].get_resolution()[1],
                                                           focalx=cam["focal"], focaly=cam["focal"],
                                                           alivetime=time.time(), depthFactor=1.0,
                                                           depth=depth.tobytes())

        self.mutex_c.acquire()
        self.cameras_write, self.cameras_read = self.cameras_read, self.cameras_write
        self.mutex_c.release()

            # try:
            #    self.camerargbdsimplepub_proxy.pushRGBD(cam["rgb"], cam["depth"])
            # except Ice.Exception as e:
            #    print(e)

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata:  # and (time.time() - self.joystick_newdata[1]) > 0.1:
            datos = self.joystick_newdata[0]
            adv = 0.0
            rot = 0.0
            for x in datos.axes:
                if x.name == "advance":
                    adv = x.value if np.abs(x.value) > 10 else 0
                if x.name == "rotate" or x.name == "turn":
                    rot = x.value if np.abs(x.value) > 0.01 else 0

            converted = self.convert_base_speed_to_motors_speed(adv, rot)
            print("Joystick ", [adv, rot], converted)
            self.joystick_newdata = None
            self.last_received_data_time = time.time()
        else:
            elapsed = time.time() - self.last_received_data_time
            if elapsed > 2 and elapsed < 3:
                self.convert_base_speed_to_motors_speed(0, 0)

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
        return left_vel, right_vel

    ###########################################
    ### ROBOT POSE get and publish robot position
    ###########################################
    def read_robot_pose(self):
        slam_0 = Shape("slam_0")
        pose = slam_0.get_position()
        rot = slam_0.get_orientation()
        linear_vel, ang_vel = slam_0.get_velocity()

        isMoving = np.abs(linear_vel[0]) > 0.01 or np.abs(linear_vel[1]) > 0.01 or np.abs(ang_vel[2]) > 0.01
        self.bState = RoboCompGenericBase.TBaseState(x=pose[0] * 1000,
                                                     z=pose[1] * 1000,
                                                     alpha=rot[2]- np.pi,
                                                     advVx=linear_vel[0] * 1000,
                                                     advVz=linear_vel[1] * 1000,
                                                     rotV=ang_vel[2],
                                                     isMoving=isMoving)

        # self.tm.add_transform("world", "robot", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #                                                             ([rot[0], rot[1], rot[2]-np.pi]),
        #                                                             [pose[0]*1000.0, pose[1]*1000.0, pose[2]*1000.0]
        #                                                             ))
        #
        # t = self.tm.get_transform("origin", "robot")
        # angles = pyrot.extrinsic_euler_xyz_from_active_matrix(t[0:3, 0:3])

        self.robot_full_pose_write.x = pose[0] * 1000
        self.robot_full_pose_write.y = pose[1] * 1000
        self.robot_full_pose_write.z = pose[2] * 1000
        self.robot_full_pose_write.rx = rot[0]
        self.robot_full_pose_write.ry = rot[1]
        self.robot_full_pose_write.rz = rot[2] - np.pi
        self.robot_full_pose_write.vx = linear_vel[0] * 1000.0
        self.robot_full_pose_write.vy = linear_vel[1] * 1000.0
        self.robot_full_pose_write.vz = linear_vel[2] * 1000.0
        self.robot_full_pose_write.vrx = ang_vel[0]
        self.robot_full_pose_write.vry = ang_vel[1]
        self.robot_full_pose_write.vrz = ang_vel[2]

        # swap
        self.mutex.acquire()
        self.robot_full_pose_write, self.robot_full_pose_read = self.robot_full_pose_read, self.robot_full_pose_write
        self.mutex.release()


    ###########################################
    ### MOVE ROBOT from Omnirobot interface
    ###########################################
    def move_robot(self):

        if self.speed_robot: #!= self.speed_robot_ant:  # or (isMoving and self.speed_robot == [0,0,0]):
            self.convert_base_speed_to_motors_speed(self.speed_robot[0], self.speed_robot[1])
            # print("Velocities sent to robot:", self.speed_robot)
            #self.speed_robot_ant = self.speed_robot
            self.speed_robot = None

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
        if camera in self.cameras_read.keys():
            return RoboCompCameraRGBDSimple.TRGBD(self.cameras_read[camera]["rgb"], self.cameras_read[camera]["depth"])
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name: " + camera
            raise e

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self, camera):
        if camera in self.cameras_read.keys():
            return self.cameras_read[camera]["depth"]
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name: " + camera
            raise e

    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self, camera):
        if camera in self.cameras_read.keys():
            return self.cameras_read[camera]["rgb"]
        else:
            e = RoboCompCameraRGBDSimple.HardwareFailedException()
            e.what = "No camera found with this name: " + camera
            raise e

    ##############################################
    ## Omnibase
    #############################################

    #
    # correctOdometer
    #
    def DifferentialRobot_correctOdometer(self, x, z, alpha):
        pass

    #
    # getBasePose
    #
    def DifferentialRobot_getBasePose(self):
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
    def DifferentialRobot_getBaseState(self):
        return self.bState

    #
    # resetOdometer
    #
    def DifferentialRobot_resetOdometer(self):
        pass

    #
    # setOdometer
    #
    def DifferentialRobot_setOdometer(self, state):
        pass

    #
    # setOdometerPose
    #
    def DifferentialRobot_setOdometerPose(self, x, z, alpha):
        pass

    #
    # setSpeedBase
    #
    def DifferentialRobot_setSpeedBase(self, advz, rot):
        self.speed_robot = [advz, rot]

    #
    # stopBase
    #
    def DifferentialRobot_stopBase(self):
        pass

    # ===================================================================
    # CoppeliaUtils
    # ===================================================================
    def CoppeliaUtils_addOrModifyDummy(self, type, name, pose):
        if not Dummy.exists(name):
            dummy = Dummy.create(0.1)
            # one color for each type of dummy
            if type == RoboCompCoppeliaUtils.TargetTypes.Info:
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
            # print("Coppelia ", name, pose.x/1000, pose.y/1000, pose.z/1000)
            dummy.set_position([pose.x / 1000., pose.y / 1000., pose.z / 1000.], parent_frame_object)
            dummy.set_orientation([pose.rx, pose.ry, pose.rz], parent_frame_object)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getFullPose method from FullPoseEstimation interface
    #
    def FullPoseEstimation_getFullPoseEuler(self):
        return self.robot_full_pose_read

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

    #
    # IMPLEMENTATION of setInitialPose method from FullPoseEstimation interface
    #
    def FullPoseEstimation_setInitialPose(self, x, y, z, rx, ry, rz):

        # should move robot in Coppelia to designated pose
        self.tm.add_transform("origin", "world",
                               pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([rx, ry, rz]), [x, y, z])
        )

    #
    # IMPLEMENTATION of getAllSensorDistances method from Ultrasound interface
    #
    def Ultrasound_getAllSensorDistances(self):
        ret = RoboCompUltrasound.SensorsState()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getAllSensorParams method from Ultrasound interface
    #
    def Ultrasound_getAllSensorParams(self):
        ret = RoboCompUltrasound.SensorParamsList()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getBusParams method from Ultrasound interface
    #
    def Ultrasound_getBusParams(self):
        ret = RoboCompUltrasound.BusParams()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getSensorDistance method from Ultrasound interface
    #
    def Ultrasound_getSensorDistance(self, sensor):
        ret = int()
        #
        # write your CODE here
        #
        return ret

    #
    # IMPLEMENTATION of getSensorParams method from Ultrasound interface
    #
    def Ultrasound_getSensorParams(self, sensor):
        ret = RoboCompUltrasound.SensorParams()
        #
        # write your CODE here
        #
        return ret

    # ===================================================================
    # ===================================================================
    #
    # IMPLEMENTATION of getRSSIState method from RSSIStatus interface
    #
    def RSSIStatus_getRSSIState(self):
        ret = RoboCompRSSIStatus.TRSSI()
        ret.percentage = 100;
        return ret

    #
    # IMPLEMENTATION of getBatteryState method from BatteryStatus interface
    #
    def BatteryStatus_getBatteryState(self):
        ret = RoboCompBatteryStatus.TBattery()
        ret.percentage = 100
        return ret
    #

# =============== Methods for Component Implements ==================
    # ===================================================================
    #
    # IMPLEMENTATION of getLaserAndBStateData method from Laser interface
    #
    def Laser_getLaserAndBStateData(self):
        ret = RoboCompLaser.TLaserData()
        bState = RoboCompGenericBase.TBaseState()
        return [ret, bState]
    #
    # IMPLEMENTATION of getLaserConfData method from Laser interface
    #
    def Laser_getLaserConfData(self):
        ret = RoboCompLaser.LaserConfData()
        return ret
    #
    # IMPLEMENTATION of getLaserData method from Laser interface
    #
    def Laser_getLaserData(self):
        return self.ldata_read
    # ===================================================================
    # ===================================================================