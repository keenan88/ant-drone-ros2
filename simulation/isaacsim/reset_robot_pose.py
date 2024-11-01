import omni.kit.commands
from pxr import Gf
from omni.isaac.core.prims import XFormPrim
import numpy as np

def setup(db: og.Database):
    pass

def compute(db: og.Database):

    prim = XFormPrim("/World/Map/Odom/linorobot2_mecanum")

    prim.set_local_pose(translation = [0., 0., 0.], orientation = [0., 0., 0., 1.])

    #return 0

    #return 1


def cleanup(db: og.Database):
    pass