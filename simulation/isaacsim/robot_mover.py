import omni.kit.commands
from pxr import Gf
from omni.isaac.core.prims import XFormPrim
import numpy as np

def setup(db: og.Database):
    pass    

def compute(db: og.Database):

    prim = XFormPrim("/World/Map/linorobot2_mecanum")

    desired_translation = db.inputs.desired_translation
    desired_orientation = db.inputs.desired_orientation

    if not np.array_equal(desired_orientation, [0., 0., 0., 0.]):
        prim.set_local_pose(translation = desired_translation, orientation = desired_orientation)


def cleanup(db: og.Database):
    pass