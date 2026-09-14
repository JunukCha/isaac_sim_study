import sys
from pathlib import Path

# Add python_scripts directory to Python module search path
PYTHON_SCRIPTS_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PYTHON_SCRIPTS_DIR))

from utils.live_stream import simulation_app
# from isaacsim.core.simulation_manager import SimulationManager

import numpy as np

from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
    interface_config_loader,
)

import time

# Isaac Sim modules must be imported after SimulationApp
from isaacsim.storage.native import get_assets_root_path
import omni.usd
import carb

from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdGeom, UsdLux, UsdPhysics, Gf

# ---------------------------------------------------------
# Get stage
# ---------------------------------------------------------
stage = omni.usd.get_context().get_stage()


# ---------------------------------------------------------
# Helper functions
# ---------------------------------------------------------
def create_cube(path, position, scale, color):
    cube = UsdGeom.Cube.Define(stage, path)

    cube.CreateSizeAttr(1.0)

    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(*scale))

    cube.CreateDisplayColorAttr(
        [Gf.Vec3f(*color)]
    )

    return cube


def create_cylinder(path, position, radius, height, color):
    cylinder = UsdGeom.Cylinder.Define(stage, path)

    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.CreateAxisAttr("Z")

    xform = UsdGeom.Xformable(cylinder)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))

    cylinder.CreateDisplayColorAttr(
        [Gf.Vec3f(*color)]
    )

    return cylinder

def open_gripper():
    gripper_action = ArticulationAction(
        joint_positions=np.array([0.04, 0.04]),
        joint_indices=np.array([7, 8]),
    )
    franka.apply_action(gripper_action)


def close_gripper():
    gripper_action = ArticulationAction(
        joint_positions=np.array([0.0, 0.0]),
        joint_indices=np.array([7, 8]),
    )
    franka.apply_action(gripper_action)

# ---------------------------------------------------------
# Scene root
# ---------------------------------------------------------
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, "/World/Scene")

# ---------------------------------------------------------
# Physics scene
# ---------------------------------------------------------
UsdPhysics.Scene.Define(
    stage,
    "/World/PhysicsScene",
)

# ---------------------------------------------------------
# Ground
# ---------------------------------------------------------
create_cube(
    "/World/Scene/Ground",
    position=(0.0, 0.0, -0.05),
    scale=(6.0, 6.0, 0.1),
    color=(0.3, 0.3, 0.3),
)

# ---------------------------------------------------------
# Cup
# ---------------------------------------------------------
cup_height = 0.18

create_cylinder(
    "/World/Scene/Cup",
    position=(
        0.15,
        0.20,
        cup_height / 2,
    ),
    radius=0.06,
    height=cup_height,
    color=(0.15, 0.35, 0.9),
)


# ---------------------------------------------------------
# Obstacle box
# ---------------------------------------------------------
box_size = (0.16, 0.16, 0.18)

create_cube(
    "/World/Scene/ObstacleBox",
    position=(
        0.28,
        -0.15,
        box_size[2] / 2,
    ),
    scale=box_size,
    color=(0.9, 0.55, 0.1),
)


# ---------------------------------------------------------
# Manipulation target
# ---------------------------------------------------------
target_size = (0.10, 0.10, 0.10)

create_cube(
    "/World/Scene/Target",
    position=(
        0.50,
        0.05,
        target_size[2] / 2,
    ),
    scale=target_size,
    color=(0.1, 0.85, 0.2),
)

# ---------------------------------------------------------
# Add Franka
# ---------------------------------------------------------
assets_root = get_assets_root_path()

franka_usd = (
    assets_root
    + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)

franka_prim = stage.DefinePrim("/World/Franka", "Xform")
franka_prim.GetReferences().AddReference(franka_usd)

# Place Franka
franka_xform = UsdGeom.Xformable(franka_prim)
translate_op = None

for op in franka_xform.GetOrderedXformOps():
    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
        translate_op = op
        break

if translate_op is None:
    translate_op = franka_xform.AddTranslateOp()

translate_op.Set(
    Gf.Vec3d(0.0, 0.0, 0.0)
)
# franka_xform.AddTranslateOp().Set(
#     Gf.Vec3d(-0.35, 0.0, 0.0)
# )

carb.log_warn("Franka added.")

# ---------------------------------------------------------
# Start physics
# ---------------------------------------------------------
SimulationManager.initialize_physics()

simulation_app.update()

# ---------------------------------------------------------
# Initialize Franka articulation
# ---------------------------------------------------------
franka = Articulation("/World/Franka")

physics_sim_view = SimulationManager.get_physics_simulation_view()

print("physics_sim_view:", physics_sim_view)

franka.initialize(
    physics_sim_view=physics_sim_view
)

carb.log_warn("Franka articulation initialized.")

# ---------------------------------------------------------
# Initialize IK solver
# ---------------------------------------------------------
kinematics_config = (
    interface_config_loader
    .load_supported_lula_kinematics_solver_config("Franka")
)

kinematics_solver = LulaKinematicsSolver(
    **kinematics_config
)

articulation_ik = ArticulationKinematicsSolver(
    franka,
    kinematics_solver,
    "right_gripper",
)

carb.log_warn("Franka IK initialized.")

#
#
#
target_prim = stage.GetPrimAtPath("/World/Scene/Target")
target_xform = UsdGeom.Xformable(target_prim)

def get_world_position(prim):
    xform_cache = UsdGeom.XformCache()
    world_transform = xform_cache.GetLocalToWorldTransform(prim)
    translation = world_transform.ExtractTranslation()

    return np.array([
        translation[0],
        translation[1],
        translation[2],
    ])

# ---------------------------------------------------------
# Lighting
# ---------------------------------------------------------
light = UsdLux.DistantLight.Define(
    stage,
    "/World/Scene/Light",
)

light.CreateIntensityAttr(3000.0)

light_xform = UsdGeom.Xformable(light)

light_xform.AddRotateXYZOp().Set(
    Gf.Vec3f(-45.0, 30.0, 0.0)
)


carb.log_warn(
    "3D Scene-Guided Manipulation scene created."
)


# ---------------------------------------------------------
# Main loop
# ---------------------------------------------------------
state = "PRE_GRASP"
last_log_time = time.time()

while simulation_app.is_running():

    target_position = get_world_position(target_prim)

    if state == "PRE_GRASP":
        desired_position = target_position.copy()
        desired_position[2] += 0.20

    elif state == "DESCEND":
        desired_position = target_position.copy()
        desired_position[2] += 0.08

    elif state == "CLOSE":
        desired_position = target_position.copy()
        desired_position[2] += 0.08
        close_gripper()

    elif state == "LIFT":
        desired_position = target_position.copy()
        desired_position[2] += 1.00

    elif state == "DONE":
        desired_position = target_position.copy()
        desired_position[2] += 1.00
        open_gripper()

    # Update robot base pose
    robot_position, robot_orientation = franka.get_world_pose()

    kinematics_solver.set_robot_base_pose(
        robot_position,
        robot_orientation,
    )

    action, success = articulation_ik.compute_inverse_kinematics(
        target_position=desired_position,
    )

    if success:
        franka.apply_action(action)
    
    ee_position, ee_orientation = articulation_ik.compute_end_effector_pose()

    distance = np.linalg.norm(
        ee_position - desired_position
    )

    if state == "PRE_GRASP" and distance < 0.02:
        state = "DESCEND"

    elif state == "DESCEND" and distance < 0.02:
        state = "CLOSE"
        close_start_time = time.time()

    elif state == "CLOSE":
        # Keep the arm at the grasp pose
        if time.time() - close_start_time > 1.0:
            state = "LIFT"

    elif state == "LIFT" and distance < 0.02:
        state = "DONE"
        done_start_time = time.time()

    elif state == "DONE":
        if distance < 0.02 and time.time() - done_start_time > 3.0:
            state = "PRE_GRASP"

    current_time = time.time()

    if current_time - last_log_time >= 2.0:
        carb.log_warn(
            f"[State] {state} | Distance: {distance:.4f} m"
        )
        last_log_time = current_time

    simulation_app.update()

# ---------------------------------------------------------
# Shutdown
# ---------------------------------------------------------
simulation_app.close()