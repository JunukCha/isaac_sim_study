import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np

from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.storage.native import get_assets_root_path

import carb


# carb.log_info("Start!")

# assets_root_path = get_assets_root_path()
# carb.log_info("assets_root_path: " + assets_root_path)


# # ---------------------------------------------------------
# # Ground
# # ---------------------------------------------------------
# stage_utils.add_reference_to_stage(
#     usd_path=assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
#     path="/World/Ground",
# )


# # ---------------------------------------------------------
# # Franka
# # ---------------------------------------------------------
# stage_utils.add_reference_to_stage(
#     usd_path=assets_root_path
#     + "/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda",
#     path="/World/Franka",
# )

# franka_xform = XformPrim(
#     "/World/Franka",
#     reset_xform_op_properties=True,
# )

# franka_xform.set_world_poses(
#     positions=np.array([[0.0, 0.0, 0.0]])
# )


# # ---------------------------------------------------------
# # Cube
# # ---------------------------------------------------------
# cube = Cube(
#     "/World/Cube",
#     positions=np.array([[0.45, 0.0, 0.025]]),
#     scales=np.array([[0.05, 0.05, 0.05]]),
#     colors=np.array([[0.0, 0.0, 1.0]]),
# )


# carb.log_info("Scene created!")
# carb.log_info("Franka: /World/Franka")
# carb.log_info("Cube: /World/Cube")

