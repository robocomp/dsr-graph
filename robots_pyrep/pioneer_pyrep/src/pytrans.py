#!/usr/bin/python3

import numpy as np
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot

tm = TransformManager()
# place the world with rotation and translationa as seen from origin
tm.add_transform("origin", "world", pytr.transform_from(pyrot.active_matrix_from_extrinsic_euler_xyz([0, 0, 0]), [0.0, 0.0, 0]))
# place the robot with rotation and translation as seen from world
tm.add_transform("world", "robot", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0, 0, np.pi/2.0]), [-10.0, 0, 0]))

t = tm.get_transform("world", "robot")
print(t)
# transform the nose of the robot to world CS
p = pytr.transform(t, np.array([0, 10, 0, 1.0]))
print(p)



