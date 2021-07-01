import numpy as np
import matplotlib.pyplot as plt
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager


world2robot = pt.transform_from(
    pr.active_matrix_from_intrinsic_euler_xyz(np.array([0.0, 0.0, -np.pi/2])),
    np.array([0, -0.5, 0.0]))

tm = TransformManager()
tm.add_transform("robot", "world", world2robot)
rt = tm.get_transform("robot", "world")
p = pt.transform(rt, np.array([0, 0.5, 0, 1.0]))
print(p)
ax = tm.plot_frames_in("world", s=0.1)
ax.scatter(p[0], p[1], p[2])
ax.set_xlim((-1, 1))
ax.set_ylim((-1, 1))
ax.set_zlim((0, 0.5))
plt.show()