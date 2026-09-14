"""
Franka IK Manipulation Demo with Isaac Sim Livestream

This example demonstrates a simple state-based manipulation pipeline
using the Franka Panda robot and Lula inverse kinematics.

Pipeline:
    PRE_GRASP -> DESCEND -> CLOSE -> LIFT -> DONE

Note:
    This example controls the Franka end-effector and gripper using IK.
    The target object is currently a visual object without rigid-body
    dynamics, so it will not physically move with the gripper.
"""

import sys
import time
from enum import Enum
from pathlib import Path

import numpy as np


# -----------------------------------------------------------------------------
# Project path
# -----------------------------------------------------------------------------

# Add python_scripts directory to the Python module search path
PYTHON_SCRIPTS_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PYTHON_SCRIPTS_DIR))

# -----------------------------------------------------------------------------
# Start Isaac Sim
# -----------------------------------------------------------------------------

# SimulationApp must be created before importing most Isaac Sim modules
from utils.live_stream import simulation_app

# -----------------------------------------------------------------------------
# Isaac Sim imports
# -----------------------------------------------------------------------------

import carb
import omni.usd

from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
    interface_config_loader,
)
from isaacsim.storage.native import get_assets_root_path


# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

FRANKA_PATH = "/World/Franka"
TARGET_PATH = "/World/Scene/Target"

END_EFFECTOR_FRAME = "right_gripper"

GRIPPER_JOINT_INDICES = np.array([7, 8])
GRIPPER_OPEN_POSITION = np.array([0.04, 0.04])
GRIPPER_CLOSED_POSITION = np.array([0.0, 0.0])

PRE_GRASP_HEIGHT = 0.20
GRASP_HEIGHT = 0.08
LIFT_HEIGHT = 1.00

POSITION_THRESHOLD = 0.02

GRIPPER_CLOSE_DELAY = 1.0
DONE_WAIT_TIME = 3.0

LOG_INTERVAL = 2.0


# -----------------------------------------------------------------------------
# Manipulation state
# -----------------------------------------------------------------------------

class ManipulationState(Enum):
    PRE_GRASP = "PRE_GRASP"
    DESCEND = "DESCEND"
    CLOSE = "CLOSE"
    LIFT = "LIFT"
    DONE = "DONE"


# -----------------------------------------------------------------------------
# Stage
# -----------------------------------------------------------------------------

stage = omni.usd.get_context().get_stage()


# -----------------------------------------------------------------------------
# Scene helper functions
# -----------------------------------------------------------------------------

def create_cube(path, position, scale, color):
    """Create a simple colored cube."""
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
    """Create a simple colored cylinder."""
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


def get_world_position(prim):
    """Return the world-space position of a USD prim."""
    xform_cache = UsdGeom.XformCache()
    world_transform = xform_cache.GetLocalToWorldTransform(prim)
    translation = world_transform.ExtractTranslation()

    return np.array(
        [
            translation[0],
            translation[1],
            translation[2],
        ],
        dtype=np.float64,
    )


# -----------------------------------------------------------------------------
# Scene setup
# -----------------------------------------------------------------------------

UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, "/World/Scene")


# -----------------------------------------------------------------------------
# Physics scene
# -----------------------------------------------------------------------------

UsdPhysics.Scene.Define(
    stage,
    "/World/PhysicsScene",
)


# -----------------------------------------------------------------------------
# Ground
# -----------------------------------------------------------------------------

create_cube(
    "/World/Scene/Ground",
    position=(0.0, 0.0, -0.05),
    scale=(6.0, 6.0, 0.1),
    color=(0.3, 0.3, 0.3),
)


# -----------------------------------------------------------------------------
# Scene objects
# -----------------------------------------------------------------------------

# Cup
cup_height = 0.18

create_cylinder(
    "/World/Scene/Cup",
    position=(0.15, 0.20, cup_height / 2),
    radius=0.06,
    height=cup_height,
    color=(0.15, 0.35, 0.9),
)


# Obstacle box
box_size = (0.16, 0.16, 0.18)

create_cube(
    "/World/Scene/ObstacleBox",
    position=(0.28, -0.15, box_size[2] / 2),
    scale=box_size,
    color=(0.9, 0.55, 0.1),
)


# Manipulation target
target_size = (0.10, 0.10, 0.10)

create_cube(
    TARGET_PATH,
    position=(0.50, 0.05, target_size[2] / 2),
    scale=target_size,
    color=(0.1, 0.85, 0.2),
)


# -----------------------------------------------------------------------------
# Franka setup
# -----------------------------------------------------------------------------

assets_root = get_assets_root_path()

franka_usd = (
    assets_root
    + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)

franka_prim = stage.DefinePrim(
    FRANKA_PATH,
    "Xform",
)

franka_prim.GetReferences().AddReference(
    franka_usd
)


# Set Franka base position
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

carb.log_warn("Franka robot added.")


# -----------------------------------------------------------------------------
# Lighting
# -----------------------------------------------------------------------------

light = UsdLux.DistantLight.Define(
    stage,
    "/World/Scene/Light",
)

light.CreateIntensityAttr(3000.0)

light_xform = UsdGeom.Xformable(light)

light_xform.AddRotateXYZOp().Set(
    Gf.Vec3f(-45.0, 30.0, 0.0)
)


# -----------------------------------------------------------------------------
# Initialize physics
# -----------------------------------------------------------------------------

SimulationManager.initialize_physics()

# Update once so that the newly created USD prims are registered
simulation_app.update()


# -----------------------------------------------------------------------------
# Initialize Franka articulation
# -----------------------------------------------------------------------------

franka = Articulation(
    FRANKA_PATH
)

physics_sim_view = SimulationManager.get_physics_simulation_view()

franka.initialize(
    physics_sim_view=physics_sim_view
)

carb.log_warn(
    "Franka articulation initialized."
)


# -----------------------------------------------------------------------------
# Gripper control
# -----------------------------------------------------------------------------

def set_gripper_position(joint_positions):
    """Set the desired position of the two finger joints."""
    action = ArticulationAction(
        joint_positions=joint_positions,
        joint_indices=GRIPPER_JOINT_INDICES,
    )

    franka.apply_action(action)


def open_gripper():
    """Open the Franka gripper."""
    set_gripper_position(
        GRIPPER_OPEN_POSITION
    )


def close_gripper():
    """Close the Franka gripper."""
    set_gripper_position(
        GRIPPER_CLOSED_POSITION
    )


# -----------------------------------------------------------------------------
# Initialize Lula IK solver
# -----------------------------------------------------------------------------

kinematics_config = (
    interface_config_loader
    .load_supported_lula_kinematics_solver_config(
        "Franka"
    )
)

kinematics_solver = LulaKinematicsSolver(
    **kinematics_config
)

articulation_ik = ArticulationKinematicsSolver(
    franka,
    kinematics_solver,
    END_EFFECTOR_FRAME,
)

carb.log_warn(
    "Franka Lula IK solver initialized."
)


# -----------------------------------------------------------------------------
# Target
# -----------------------------------------------------------------------------

target_prim = stage.GetPrimAtPath(
    TARGET_PATH
)

if not target_prim.IsValid():
    raise RuntimeError(
        f"Target prim not found: {TARGET_PATH}"
    )


# -----------------------------------------------------------------------------
# Initial state
# -----------------------------------------------------------------------------

state = ManipulationState.PRE_GRASP

state_start_time = time.monotonic()
last_log_time = time.monotonic()

open_gripper()

carb.log_warn(
    "3D scene-guided manipulation demo started."
)


# -----------------------------------------------------------------------------
# Main simulation loop
# -----------------------------------------------------------------------------

while simulation_app.is_running():

    # -------------------------------------------------------------------------
    # Get current target position
    # -------------------------------------------------------------------------

    target_position = get_world_position(
        target_prim
    )


    # -------------------------------------------------------------------------
    # Define the desired end-effector position for each state
    # -------------------------------------------------------------------------

    if state == ManipulationState.PRE_GRASP:

        desired_position = target_position.copy()
        desired_position[2] += PRE_GRASP_HEIGHT

    elif state == ManipulationState.DESCEND:

        desired_position = target_position.copy()
        desired_position[2] += GRASP_HEIGHT

    elif state == ManipulationState.CLOSE:

        desired_position = target_position.copy()
        desired_position[2] += GRASP_HEIGHT

        close_gripper()

    elif state == ManipulationState.LIFT:

        desired_position = target_position.copy()
        desired_position[2] += LIFT_HEIGHT

    elif state == ManipulationState.DONE:

        desired_position = target_position.copy()
        desired_position[2] += LIFT_HEIGHT


    # -------------------------------------------------------------------------
    # Update the robot base pose for Lula
    # -------------------------------------------------------------------------

    robot_position, robot_orientation = (
        franka.get_world_pose()
    )

    kinematics_solver.set_robot_base_pose(
        robot_position,
        robot_orientation,
    )


    # -------------------------------------------------------------------------
    # Solve inverse kinematics
    # -------------------------------------------------------------------------

    action, success = (
        articulation_ik.compute_inverse_kinematics(
            target_position=desired_position,
        )
    )

    if success:
        franka.apply_action(action)


    # -------------------------------------------------------------------------
    # Compute current end-effector position
    # -------------------------------------------------------------------------

    ee_position, _ = (
        articulation_ik.compute_end_effector_pose()
    )

    distance = np.linalg.norm(
        ee_position - desired_position
    )


    # -------------------------------------------------------------------------
    # State transitions
    # -------------------------------------------------------------------------

    if (
        state == ManipulationState.PRE_GRASP
        and distance < POSITION_THRESHOLD
    ):
        state = ManipulationState.DESCEND
        state_start_time = time.monotonic()

    elif (
        state == ManipulationState.DESCEND
        and distance < POSITION_THRESHOLD
    ):
        state = ManipulationState.CLOSE
        state_start_time = time.monotonic()

    elif state == ManipulationState.CLOSE:

        if (
            time.monotonic() - state_start_time
            > GRIPPER_CLOSE_DELAY
        ):
            state = ManipulationState.LIFT
            state_start_time = time.monotonic()

    elif (
        state == ManipulationState.LIFT
        and distance < POSITION_THRESHOLD
    ):
        state = ManipulationState.DONE
        state_start_time = time.monotonic()

    elif state == ManipulationState.DONE:

        if (
            distance < POSITION_THRESHOLD
            and time.monotonic() - state_start_time
            > DONE_WAIT_TIME
        ):
            open_gripper()

            state = ManipulationState.PRE_GRASP
            state_start_time = time.monotonic()


    # -------------------------------------------------------------------------
    # Periodic logging
    # -------------------------------------------------------------------------

    current_time = time.monotonic()

    if current_time - last_log_time >= LOG_INTERVAL:

        carb.log_warn(
            f"[State] {state.value} | "
            f"EE distance: {distance:.4f} m | "
            f"IK: {'Success' if success else 'Failed'}"
        )

        last_log_time = current_time


    # -------------------------------------------------------------------------
    # Advance Isaac Sim
    # -------------------------------------------------------------------------

    simulation_app.update()


# -----------------------------------------------------------------------------
# Shutdown
# -----------------------------------------------------------------------------

simulation_app.close()