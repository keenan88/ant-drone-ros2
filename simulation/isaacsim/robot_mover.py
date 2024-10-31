import omni.kit.commands
from pxr import Gf
from omni.isaac.core.prims import XFormPrim
import numpy as np

def setup(db: og.Database):
    pass
    # prim = XFormPrim("/World/Cylinder")
    # prim.set_local_pose(translation=np.array([1.0, 0.5, 0.0]), orientation=np.array([1., 0., 0., 0.]))

def compute(db: og.Database):

    prim = XFormPrim("/World/Map/linorobot2_mecanum")

    # Set the new position

    desired_pos = db.inputs.desired_pos

    prim.set_local_pose(translation=desired_pos, orientation=np.array([1., 0., 0., 0.]))

    #return True

    pass
    

def cleanup(db: og.Database):
    pass