# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demonstrate Spot robot simulation with policy control.

The robot deploys through the generic :class:`RobotPolicyRunner` (bundled Spot spec, derived
binding, policy runtime) instead of the legacy ``SpotFlatTerrainPolicy`` class. Everything
term-level — including Spot's action scale — derives from the selected engine's IO descriptor,
and the spawn pose falls back to the env config's ``init_state`` through the runner's built-in
fallback chain.
"""


import sys
from pathlib import Path

# Add python_scripts directory to Python module search path
PYTHON_SCRIPTS_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PYTHON_SCRIPTS_DIR))

import argparse

from isaacsim import SimulationApp
from utils.live_stream import simulation_app
parser = argparse.ArgumentParser(description="Select simulation engine and device.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cuda", help="Simulation device")
parser.add_argument("--engine", type=str, choices=["physx", "newton"], default="physx", help="Physics engine")

args, unknown = parser.parse_known_args()
extra_args = [f"--/exts/isaacsim.core.simulation_manager/default_engine={args.engine}"]
if args.engine == "newton":
    extra_args.extend(["--enable", "isaacsim.physics.newton", "--enable", "isaacsim.physics.newton.tensors"])
# simulation_app = SimulationApp({"headless": False, "extra_args": extra_args})


import carb
import numpy as np
import omni.timeline
from command_path import TraveledPath, author_command_path, phase_boundary_frames, report_tracking
from isaacsim.core.experimental.utils.stage import define_prim, set_stage_units, set_stage_up_axis
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples import PolicyEnvConfig, RobotPolicyRunner, get_spot_spec
from isaacsim.storage.native import get_assets_root_path

first_step = True
reset_needed = False
policy_failed = False

print(f"Using engine: {args.engine}")
print(f"Using device: {args.device}")


# initialize robot on first step, run robot advance
def on_physics_step(step_size: float, context: object) -> None:
    """Initialize, reset, or advance the Spot policy runner after a physics step.

    Args:
        step_size: Duration of the completed physics step.
        context: User context supplied when the callback was registered.
    """
    global first_step, reset_needed, policy_failed
    if policy_failed:
        return
    try:
        if first_step:
            spot.restart_from_default_state(base_command)
            first_step = False
        elif reset_needed:
            reset_needed = False
            first_step = True
        else:
            spot.step(step_size, base_command)
    except Exception as error:  # noqa: BLE001 - a physics callback must not raise
        policy_failed = True
        carb.log_error(f"spot_standalone: policy deployment failed, stopping: {error}")


# spawn world
set_stage_up_axis("Z")
set_stage_units(meters_per_unit=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
prim.GetReferences().AddReference(asset_path)

# spawn physics scene
# TODO: physics scene should be created by simulation manager
define_prim("/World/PhysicsScene", "PhysicsScene")

# select the simulation device before constructing the policy articulation
SimulationManager.set_physics_sim_device(args.device)

# spawn robot through the generic policy runner (bundled Spot spec)
spec = get_spot_spec()
spot = RobotPolicyRunner(spec, prim_path="/World/Spot")
spot.spawn()

# timing from the deployed artifact's env config (render cadence comes from render_interval)
timing = PolicyEnvConfig.from_file(spec.engines[args.engine].env_config_path).timing
frame_dt = timing.render_interval * timing.physics_dt
RenderingManager.set_dt(frame_dt)
SimulationManager.set_physics_dt(spot.physics_dt)

# scripted command loop: (app frames, [vx, vy, yaw_rate]), one command held per frame. The last
# three phases turn toward the spawn point, walk back to it, and restore the spawn heading, so the
# commanded course closes on itself and the robot patrols the same circuit every lap.
COMMAND_PHASES = [
    (55, [1.5, 0.0, 0.0]),  # forward
    (30, [0.0, 1.0, 0.0]),  # strafe left
    (68, [1.2, 0.0, 1.4]),  # arc left
    (40, [1.4, 0.0, 0.0]),  # forward
    (68, [1.2, 0.0, 1.4]),  # arc left
    (25, [0.0, -0.9, 0.0]),  # strafe right
    (40, [1.3, 0.0, 0.0]),  # forward
    (68, [1.2, 0.0, 1.4]),  # arc left
    (34, [0.0, 0.0, -1.2]),  # turn toward the spawn point
    (51, [1.5, 0.0, 0.0]),  # return leg
    (58, [0.0, 0.0, 1.2]),  # turn back to the spawn heading
]
frame_commands = np.concatenate([np.tile(np.asarray(c, dtype=np.float32), (n, 1)) for n, c in COMMAND_PHASES])
phase_boundaries = phase_boundary_frames(COMMAND_PHASES)
# green: the commanded course, with a waypoint arrow per phase; red: where the robot actually went
commanded_end = author_command_path(COMMAND_PHASES, start_position=(0.0, 0.0), frame_dt=frame_dt)
traveled = TraveledPath()
# robot command
pressed_keys = set()
base_command = np.zeros(3, dtype=np.float32)

# Keyboard input
input_interface = carb.input.acquire_input_interface()
keyboard = omni.appwindow.get_default_app_window().get_keyboard()

# def on_keyboard_event(event):
#     if event.type == carb.input.KeyboardEventType.KEY_PRESS:

#         if event.input == carb.input.KeyboardInput.W:
#             base_command[:] = [1.0, 0.0, 0.0]
#             carb.log_warn("W pressed: forward")

#         elif event.input == carb.input.KeyboardInput.S:
#             base_command[:] = [-1.0, 0.0, 0.0]
#             carb.log_warn("S pressed: backward")

#         elif event.input == carb.input.KeyboardInput.A:
#             base_command[:] = [0.0, 1.0, 0.0]
#             carb.log_warn("A pressed: left")

#         elif event.input == carb.input.KeyboardInput.D:
#             base_command[:] = [0.0, -1.0, 0.0]
#             carb.log_warn("D pressed: right")

#         elif event.input == carb.input.KeyboardInput.Q:
#             base_command[:] = [0.0, 0.0, 1.0]
#             carb.log_warn("Q pressed: counter-clockwise")

#         elif event.input == carb.input.KeyboardInput.E:
#             base_command[:] = [0.0, 0.0, -1.0]
#             carb.log_warn("E pressed: clockwise")


#     elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:

#         if event.input in (
#             carb.input.KeyboardInput.W,
#             carb.input.KeyboardInput.S,
#             carb.input.KeyboardInput.A,
#             carb.input.KeyboardInput.D,
#             carb.input.KeyboardInput.Q,
#             carb.input.KeyboardInput.E,
#         ):
#             base_command[:] = [0.0, 0.0, 0.0]
#             carb.log_warn("Key released: stop")

#     return True

def update_base_command():
    # Reset command
    base_command[:] = [0.0, 0.0, 0.0]

    # Linear x
    if carb.input.KeyboardInput.W in pressed_keys:
        base_command[0] += 1.0
    if carb.input.KeyboardInput.S in pressed_keys:
        base_command[0] -= 1.0

    # Linear y
    if carb.input.KeyboardInput.A in pressed_keys:
        base_command[1] += 1.0
    if carb.input.KeyboardInput.D in pressed_keys:
        base_command[1] -= 1.0

    # Yaw rate
    if carb.input.KeyboardInput.Q in pressed_keys:
        base_command[2] += 1.0
    if carb.input.KeyboardInput.E in pressed_keys:
        base_command[2] -= 1.0

    carb.log_warn(f"Current command: {base_command}")

def on_keyboard_event(event):
    watched_keys = {
        carb.input.KeyboardInput.W,
        carb.input.KeyboardInput.A,
        carb.input.KeyboardInput.S,
        carb.input.KeyboardInput.D,
        carb.input.KeyboardInput.Q,
        carb.input.KeyboardInput.E,
    }

    if event.input not in watched_keys:
        return True

    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        pressed_keys.add(event.input)
        update_base_command()

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        pressed_keys.discard(event.input)
        update_base_command()

    return True

keyboard_sub = input_interface.subscribe_to_keyboard_events(
    keyboard,
    on_keyboard_event,
)

# register physics callback
_physics_callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

# play simulation
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

i = 0
loop_index = 0
while simulation_app.is_running():
    simulation_app.update()

    if not SimulationManager.is_simulating():
        reset_needed = True
# while simulation_app.is_running():
#     simulation_app.update()
#     if SimulationManager.is_simulating():
#         if i == len(frame_commands):
#             i = 0
#             loop_index += 1
#             if args.test is True:
#                 report_tracking(commanded_end, traveled)
#                 break
#         base_command = frame_commands[i]
#         # the commanded course describes the first circuit, so only mark its phase boundaries
#         traveled.record(spot.articulation, mark=(loop_index == 0 and i in phase_boundaries))
#         i += 1
#     else:
#         reset_needed = True
timeline.stop()
SimulationManager.deregister_callback(_physics_callback_id)
spot.close()
simulation_app.close()
