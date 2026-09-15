"""
Franka collision-aware top-down grasp demo with Isaac Sim Livestream.

Pipeline:
    PRE_GRASP -> DESCEND -> CLOSE -> LIFT -> DONE

This example extends the previous IK-based grasp demo by adding:
    - RMPflow-based arm motion
    - Static obstacle collision avoidance
    - A fixed cylindrical cup obstacle
    - A fixed box obstacle
    - The previous grasp / lift state machine
"""

import sys
import time
from enum import Enum
from pathlib import Path

import numpy as np


# -----------------------------------------------------------------------------
# Project path
# -----------------------------------------------------------------------------

PYTHON_SCRIPTS_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PYTHON_SCRIPTS_DIR))


# -----------------------------------------------------------------------------
# Start Isaac Sim
# -----------------------------------------------------------------------------

from utils.live_stream import simulation_app

import omni.kit.app


extension_manager = omni.kit.app.get_app().get_extension_manager()

extension_manager.set_extension_enabled_immediate(
    "isaacsim.code_editor.python_server",
    True,
)

print("Python server extension enabled")

# -----------------------------------------------------------------------------
# Isaac Sim imports
# -----------------------------------------------------------------------------

import json
import carb
import omni.usd

from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, UsdShade

from isaacsim.core.api.objects import FixedCuboid, FixedCylinder
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    ArticulationMotionPolicy,
    LulaKinematicsSolver,
    RmpFlow,
    interface_config_loader,
)
from isaacsim.storage.native import get_assets_root_path

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

FRANKA_PATH = "/World/Franka"
TARGET_PATH = "/World/Scene/Target"

CUP_PATH = "/World/Scene/Cup"
OBSTACLE_BOX_PATH = "/World/Scene/ObstacleBox"

END_EFFECTOR_FRAME = "right_gripper"

GRIPPER_JOINT_INDICES = np.array([7, 8])
GRIPPER_OPEN_POSITION = np.array([0.04, 0.04])
GRIPPER_CLOSED_POSITION = np.array([0.0, 0.0])

# Target size must be smaller than the approximately 8 cm gripper opening.
TARGET_SIZE = np.array([0.05, 0.05, 0.05])
TARGET_MASS = 0.10

# Static obstacle geometry.
CUP_POSITION = np.array([0.15, 0.20, 0.09])
CUP_RADIUS = 0.06
CUP_HEIGHT = 0.18

BOX_POSITION = np.array([0.28, -0.15, 0.09])
BOX_SIZE = np.array([0.16, 0.16, 0.18])

# Grasp motion parameters.
PRE_GRASP_HEIGHT = 0.20
GRASP_HEIGHT = 0.02
LIFT_HEIGHT = 0.25

POSITION_THRESHOLD = 0.02

GRIPPER_CLOSE_DELAY = 1.0
LOG_INTERVAL = 2.0

# Motion-policy timing.
PHYSICS_DT = 1.0 / 60.0

# Fixed top-down orientation for the Franka right_gripper frame.
GRASP_ORIENTATION = euler_angles_to_quats(
    np.array([0.0, np.pi, 0.0])
)

# Physics parameters.
GRAVITY_MAGNITUDE = 9.81
STATIC_FRICTION = 1.0
DYNAMIC_FRICTION = 0.8
RESTITUTION = 0.0


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

def load_camera_view(json_path):
    """Load camera pose from JSON and apply it to the viewport."""

    with open(json_path, "r") as f:
        config = json.load(f)

    camera_path = config["camera_path"]

    position = np.array(
        config["position"],
        dtype=np.float64,
    )

    orientation = config["orientation"]

    # Quaternion format: [w, x, y, z]
    quat = Gf.Quatd(
        orientation[0],
        Gf.Vec3d(
            orientation[1],
            orientation[2],
            orientation[3],
        ),
    )

    rotation = Gf.Rotation(quat)

    # USD camera looks along its local -Z axis.
    forward = rotation.TransformDir(
        Gf.Vec3d(0.0, 0.0, -1.0)
    )

    forward = np.array(
        [
            forward[0],
            forward[1],
            forward[2],
        ],
        dtype=np.float64,
    )

    # Any positive distance works because only the viewing direction matters.
    target = position + forward

    set_camera_view(
        eye=position,
        target=target,
        camera_prim_path=camera_path,
    )

    carb.log_warn(
        f"Camera view loaded from: {json_path}"
    )


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


def add_collision(prim):
    """Enable collision on a USD prim."""
    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    collision_api.CreateCollisionEnabledAttr(True)


def add_rigid_body(prim, mass):
    """Enable rigid-body dynamics and assign mass."""
    rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body_api.CreateRigidBodyEnabledAttr(True)

    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(mass)


def create_physics_material(
    path,
    static_friction,
    dynamic_friction,
    restitution,
):
    """Create a USD physics material."""
    material = UsdShade.Material.Define(stage, path)

    material_api = UsdPhysics.MaterialAPI.Apply(
        material.GetPrim()
    )

    material_api.CreateStaticFrictionAttr(
        static_friction
    )
    material_api.CreateDynamicFrictionAttr(
        dynamic_friction
    )
    material_api.CreateRestitutionAttr(
        restitution
    )

    return material


def bind_physics_material(prim, material):
    """Bind a physics material to a prim."""
    material_binding_api = UsdShade.MaterialBindingAPI.Apply(
        prim
    )

    material_binding_api.Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )


# -----------------------------------------------------------------------------
# Scene setup
# -----------------------------------------------------------------------------

UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, "/World/Scene")
UsdGeom.Xform.Define(stage, "/World/PhysicsMaterials")


# -----------------------------------------------------------------------------
# Physics scene
# -----------------------------------------------------------------------------

physics_scene = UsdPhysics.Scene.Define(
    stage,
    "/World/PhysicsScene",
)

physics_scene.CreateGravityDirectionAttr().Set(
    Gf.Vec3f(0.0, 0.0, -1.0)
)

physics_scene.CreateGravityMagnitudeAttr().Set(
    GRAVITY_MAGNITUDE
)


# -----------------------------------------------------------------------------
# Physics material
# -----------------------------------------------------------------------------

grasp_material = create_physics_material(
    "/World/PhysicsMaterials/GraspMaterial",
    static_friction=STATIC_FRICTION,
    dynamic_friction=DYNAMIC_FRICTION,
    restitution=RESTITUTION,
)


# -----------------------------------------------------------------------------
# Ground
# -----------------------------------------------------------------------------

ground = create_cube(
    "/World/Scene/Ground",
    position=(0.0, 0.0, -0.05),
    scale=(6.0, 6.0, 0.1),
    color=(0.3, 0.3, 0.3),
)

ground_prim = ground.GetPrim()

add_collision(ground_prim)
bind_physics_material(
    ground_prim,
    grasp_material,
)


# -----------------------------------------------------------------------------
# Static obstacles
# -----------------------------------------------------------------------------
#
# These are high-level Isaac Sim fixed objects rather than visual-only USD
# geometry. Therefore they have colliders and can also be registered directly
# with RMPflow.

cup = FixedCylinder(
    prim_path=CUP_PATH,
    name="cup_obstacle",
    position=CUP_POSITION,
    radius=CUP_RADIUS,
    height=CUP_HEIGHT,
    color=np.array([0.15, 0.35, 0.9]),
)

obstacle_box = FixedCuboid(
    prim_path=OBSTACLE_BOX_PATH,
    name="box_obstacle",
    position=BOX_POSITION,
    size=1.0,
    scale=BOX_SIZE,
    color=np.array([0.9, 0.55, 0.1]),
)


# -----------------------------------------------------------------------------
# Manipulation target
# -----------------------------------------------------------------------------

target = create_cube(
    TARGET_PATH,
    position=(
        0.50,
        0.05,
        TARGET_SIZE[2] / 2,
    ),
    scale=tuple(TARGET_SIZE),
    color=(0.1, 0.85, 0.2),
)

target_prim = target.GetPrim()

add_collision(target_prim)
add_rigid_body(
    target_prim,
    TARGET_MASS,
)
bind_physics_material(
    target_prim,
    grasp_material,
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


# Set Franka base position.
franka_xform = UsdGeom.Xformable(
    franka_prim
)

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

carb.log_warn(
    "Franka robot added."
)


# -----------------------------------------------------------------------------
# Lighting
# -----------------------------------------------------------------------------

light = UsdLux.DistantLight.Define(
    stage,
    "/World/Scene/Light",
)

light.CreateIntensityAttr(
    3000.0
)

light_xform = UsdGeom.Xformable(
    light
)

light_xform.AddRotateXYZOp().Set(
    Gf.Vec3f(-45.0, 30.0, 0.0)
)


# -----------------------------------------------------------------------------
# Initial camera view
# -----------------------------------------------------------------------------

CAMERA_POSE_PATH = (
    "/workspace/isaacsim/isaac_sim_study/"
    "python_scripts/projects/3_3d_manipulation/"
    "3_camera_pose.json"
)

load_camera_view(CAMERA_POSE_PATH)


# -----------------------------------------------------------------------------
# Initialize physics
# -----------------------------------------------------------------------------

SimulationManager.initialize_physics()

# Register newly created USD prims.
simulation_app.update()


# -----------------------------------------------------------------------------
# Initialize Franka articulation
# -----------------------------------------------------------------------------

franka = Articulation(
    FRANKA_PATH
)

physics_sim_view = (
    SimulationManager.get_physics_simulation_view()
)

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
# Initialize Lula kinematics solver
# -----------------------------------------------------------------------------
#
# RMPflow controls the arm. The kinematics solver is kept only to measure the
# current end-effector pose for state-transition checks.

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
    "Franka Lula kinematics solver initialized."
)


# -----------------------------------------------------------------------------
# Initialize RMPflow
# -----------------------------------------------------------------------------

rmpflow_config = (
    interface_config_loader
    .load_supported_motion_policy_config(
        "Franka",
        "RMPflow",
    )
)

rmpflow = RmpFlow(
    **rmpflow_config
)

articulation_rmpflow = ArticulationMotionPolicy(
    robot_articulation=franka,
    motion_policy=rmpflow,
    default_physics_dt=PHYSICS_DT,
)

# Register static obstacles in the RMPflow planning world.

cup_added = rmpflow.add_obstacle(cup, static=True)
box_added = rmpflow.add_obstacle(obstacle_box, static=True)

carb.log_warn(
    f"RMPflow obstacle registration | "
    f"Cup: {cup_added} | Box: {box_added}"
)


# Optional debugging:
# Visualize the collision spheres RMPflow uses for the Franka.
#
# rmpflow.visualize_collision_spheres()


# -----------------------------------------------------------------------------
# Initial manipulation target
# -----------------------------------------------------------------------------

if not target_prim.IsValid():
    raise RuntimeError(
        f"Target prim not found: {TARGET_PATH}"
    )

# Keep the grasp reference fixed during the lift.
grasp_reference_position = get_world_position(
    target_prim
)

pre_grasp_position = grasp_reference_position.copy()
pre_grasp_position[2] += PRE_GRASP_HEIGHT

grasp_position = grasp_reference_position.copy()
grasp_position[2] += GRASP_HEIGHT

lift_position = np.array([
    0.15,   # x
    -0.35,  # y
    0.30,   # z
])


# -----------------------------------------------------------------------------
# Episode reset
# -----------------------------------------------------------------------------

def reset_episode():
    global state
    global state_start_time
    global last_log_time

    state = ManipulationState.PRE_GRASP

    state_start_time = time.monotonic()
    last_log_time = time.monotonic()

    rmpflow.reset()

    rmpflow.add_obstacle(cup, static=True)
    rmpflow.add_obstacle(obstacle_box, static=True)

    open_gripper()

    carb.log_warn(
        "Episode reset."
    )


# -----------------------------------------------------------------------------
# Initial state
# -----------------------------------------------------------------------------

state = ManipulationState.PRE_GRASP

state_start_time = time.monotonic()
last_log_time = time.monotonic()

open_gripper()

carb.log_warn(
    "Collision-aware grasp demo started."
)

carb.log_warn(
    f"Target reference position: {grasp_reference_position}"
)

carb.log_warn(
    f"Fixed grasp orientation: {GRASP_ORIENTATION}"
)


# -----------------------------------------------------------------------------
# Main simulation loop
# -----------------------------------------------------------------------------

reset_needed = False

while simulation_app.is_running():

    # -------------------------------------------------------------------------
    # Handle GUI Stop
    # -------------------------------------------------------------------------

    if not SimulationManager.is_simulating():
        reset_needed = True
        simulation_app.update()
        continue


    # -------------------------------------------------------------------------
    # Handle GUI Play after Stop
    # -------------------------------------------------------------------------

    if reset_needed:

        carb.log_warn(
            "Simulation restarted. Reinitializing Franka..."
        )

        simulation_app.update()

        physics_sim_view = (
            SimulationManager.get_physics_simulation_view()
        )

        franka.initialize(
            physics_sim_view=physics_sim_view
        )

        reset_episode()

        reset_needed = False

        carb.log_warn(
            "Franka reinitialized."
        )


    # -------------------------------------------------------------------------
    # Select target pose for the current state
    # -------------------------------------------------------------------------

    if state == ManipulationState.PRE_GRASP:
        desired_position = pre_grasp_position

    elif state == ManipulationState.DESCEND:
        desired_position = grasp_position

    elif state == ManipulationState.CLOSE:
        desired_position = grasp_position
        close_gripper()

    elif state == ManipulationState.LIFT:
        desired_position = lift_position

    else:
        desired_position = lift_position


    # -------------------------------------------------------------------------
    # Update robot-base pose
    # -------------------------------------------------------------------------

    robot_position, robot_orientation = (
        franka.get_world_pose()
    )

    # Lula kinematics solver uses the base pose for EE pose measurement.
    kinematics_solver.set_robot_base_pose(
        robot_position,
        robot_orientation,
    )

    # RMPflow also needs the robot base pose.
    rmpflow.set_robot_base_pose(
        robot_position,
        robot_orientation,
    )


    # -------------------------------------------------------------------------
    # Update RMPflow target and world
    # -------------------------------------------------------------------------

    rmpflow.set_end_effector_target(
        target_position=desired_position,
        target_orientation=GRASP_ORIENTATION,
    )

    # This is inexpensive here because the obstacles are static, but keeping
    # the call makes the code easy to extend to moving obstacles later.
    rmpflow.update_world()


    # -------------------------------------------------------------------------
    # Compute collision-aware arm action
    # -------------------------------------------------------------------------

    arm_action = (
        articulation_rmpflow
        .get_next_articulation_action(
            PHYSICS_DT
        )
    )

    franka.apply_action(
        arm_action
    )


    # -------------------------------------------------------------------------
    # Compute current end-effector position
    # -------------------------------------------------------------------------

    ee_position, _ = (
        articulation_ik.compute_end_effector_pose()
    )

    if ee_position is None:
        carb.log_warn(
            "End-effector pose is not available. Skipping frame."
        )
        simulation_app.update()
        continue

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

        carb.log_warn(
            "Transition: PRE_GRASP -> DESCEND"
        )

    elif (
        state == ManipulationState.DESCEND
        and distance < POSITION_THRESHOLD
    ):
        state = ManipulationState.CLOSE
        state_start_time = time.monotonic()

        carb.log_warn(
            "Transition: DESCEND -> CLOSE"
        )

    elif state == ManipulationState.CLOSE:

        if (
            time.monotonic() - state_start_time
            > GRIPPER_CLOSE_DELAY
        ):
            state = ManipulationState.LIFT
            state_start_time = time.monotonic()

            carb.log_warn(
                "Transition: CLOSE -> LIFT"
            )

    elif (
        state == ManipulationState.LIFT
        and distance < POSITION_THRESHOLD
    ):
        state = ManipulationState.DONE
        state_start_time = time.monotonic()

        carb.log_warn(
            "Transition: LIFT -> DONE"
        )


    # -------------------------------------------------------------------------
    # Periodic logging
    # -------------------------------------------------------------------------

    current_time = time.monotonic()

    if current_time - last_log_time >= LOG_INTERVAL:

        current_target_position = get_world_position(
            target_prim
        )

        carb.log_warn(
            f"[State] {state.value} | "
            f"EE distance: {distance:.4f} m | "
            f"Target z: {current_target_position[2]:.4f} m"
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
