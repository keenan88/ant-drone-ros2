import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.articulations import Articulation

prim_path = "/World/Map/Odom/linorobot2_mecanum/base_footprint"

prim = Articulation(prim_path=prim_path, name="base_footprint")

linear_velocity = [10.0, 0.0, 0.0]

angular_velocity = [0.0, 0.0, 10.0]

# while 1:
#     prim.set_world_velocity(linear_velocity + angular_velocity)
    