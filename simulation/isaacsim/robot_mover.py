import omni.kit.commands
from pxr import Gf
from omni.isaac.core.prims import XFormPrim
import numpy as np
import time

global N_tfs
N_tfs = 0

def setup(db: og.Database):
    pass    

def compute(db: og.Database):

    prim = XFormPrim("/World/Map/Odom/linorobot2_mecanum")

    desired_translation = db.inputs.desired_translation
    desired_orientation = db.inputs.desired_orientation

    if not np.array_equal(desired_orientation, [0., 0., 0., 0.]):
        prim.set_world_pose(position = desired_translation, orientation = desired_orientation)
        global N_tfs

        N_tfs += 1

    return True


def cleanup(db: og.Database):
    pass