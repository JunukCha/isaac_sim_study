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

from isaacsim.core.prims import SingleArticulation
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
    interface_config_loader,
)


# ---------------------------------------------------------
# 1. Get Franka articulation
# ---------------------------------------------------------
franka = SingleArticulation("/World/Franka")

franka.initialize()


# ---------------------------------------------------------
# 2. Load Franka IK configuration
# ---------------------------------------------------------
kinematics_config = (
    interface_config_loader
    .load_supported_lula_kinematics_solver_config("Franka")
)

ik_solver = LulaKinematicsSolver(**kinematics_config)


# ---------------------------------------------------------
# 3. Connect IK solver to Franka
# ---------------------------------------------------------
articulation_ik = ArticulationKinematicsSolver(
    franka,
    ik_solver,
    "panda_hand",
)


# ---------------------------------------------------------
# 4. Target position
# ---------------------------------------------------------
target_position = np.array([
    0.45,
    0.0,
    0.2,
])


# ---------------------------------------------------------
# 5. Solve inverse kinematics
# ---------------------------------------------------------
action, success = articulation_ik.compute_inverse_kinematics(
    target_position=target_position,
)


# ---------------------------------------------------------
# 6. Apply joint targets
# ---------------------------------------------------------
if success:
    carb.log_warn("IK succeeded")

    controller = franka.get_articulation_controller()
    controller.apply_action(action)

else:
    carb.log_warn("IK failed")