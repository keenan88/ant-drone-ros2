from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf
import carb
import numpy as np

new_position = Gf.Vec3f(
    1.0,
    0,
    0
)

prim = XFormPrim("/World/Cylinder")

# Set the new position
# prim.set_local_pose(translation=np.array([1.0, 0.5, 0.0]), orientation=np.array([1., 0., 0., 0.]))

prim.set_velocities(np.array([1.0, 0.0, 0.0]))