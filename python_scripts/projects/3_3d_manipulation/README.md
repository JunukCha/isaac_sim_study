# 3D Manipulation

[한국어](README.ko.md) · [Back to root](../../../README.md)

Franka Panda manipulation experiments using the Lula inverse kinematics solver, from end-effector motion to physics-based grasping and release.

## Examples

| Script | Purpose |
| --- | --- |
| [`1_move_franka.py`](1_move_franka.py) | Basic IK motion and gripper control with a visual target |
| [`1_test_code.py`](1_test_code.py) | Earlier experimental version retained for comparison and debugging |
| [`2_grasp_object.py`](2_grasp_object.py) | Approach from above, close the gripper, and lift a rigid-body target; keep the gripper closed at completion |
| [`2_test_code.py`](2_test_code.py) | Grasp-and-lift variant that opens the gripper at completion to release the target |

In `1_move_franka.py`, the target is a visual object without rigid-body dynamics, so closing the gripper does not make it move with the robot.

## Run

Run one script at a time from the repository root with Isaac Sim's bundled Python:

```powershell
# Windows: choose an example
& "<ISAAC_SIM>\python.bat" python_scripts/projects/3_3d_manipulation/1_move_franka.py
& "<ISAAC_SIM>\python.bat" python_scripts/projects/3_3d_manipulation/2_grasp_object.py
& "<ISAAC_SIM>\python.bat" python_scripts/projects/3_3d_manipulation/2_test_code.py
```

```bash
# Linux: grasp example; replace the filename to run another example
<ISAAC_SIM>/python.sh python_scripts/projects/3_3d_manipulation/2_grasp_object.py
```

The examples use the shared `utils.live_stream` module to start a headless Isaac Sim Livestream session. See the [utilities guide](../../utils/README.md) for startup and network diagnostics. Isaac Sim Assets access is required to load Franka.

## Grasp and release behavior

Both `2_*.py` scripts control the end effector and gripper through:

```text
PRE_GRASP -> DESCEND -> CLOSE -> LIFT -> DONE
```

They add a fixed top-down end-effector orientation, ground and target collision, target rigid-body dynamics and mass, gravity, and friction materials on the ground and target. The grasp and lift positions are computed from a fixed target reference so the commanded lift position does not rise along with the object.

In `DONE`, `2_grasp_object.py` holds the lift pose with the gripper closed. `2_test_code.py` holds the same pose and calls `open_gripper()` to release the object; it does not move to a separate placement location.

State transitions depend on end-effector position and the gripper closing delay, rather than detected grasp success. Check the simulation and periodic logs (`State`, `EE distance`, `Target z`, and `IK`) to assess whether the object was lifted. These descriptions reflect the code; successful physical grasping has not been verified for this documentation update.

## Main grasp parameters

Edit the constants near the top of the selected `2_*.py` script; both currently use these defaults:

| Constant | Default | Meaning |
| --- | --- | --- |
| `TARGET_SIZE` | `[0.05, 0.05, 0.05]` m | Target dimensions, smaller than the approximately 0.08 m gripper opening |
| `TARGET_MASS` | `0.10` kg | Target mass |
| `PRE_GRASP_HEIGHT` | `0.20` m | Approach height above the target reference |
| `GRASP_HEIGHT` | `0.02` m | Grasp height above the target reference |
| `LIFT_HEIGHT` | `0.25` m | Lift distance above the grasp position |
| `POSITION_THRESHOLD` | `0.02` m | Position tolerance for motion state transitions |
| `GRIPPER_CLOSE_DELAY` | `1.0` s | Delay before lifting after the close command |
| `GRASP_ORIENTATION` | Euler `[0, pi, 0]` converted to a quaternion | Fixed top-down orientation |
| `GRAVITY_MAGNITUDE` | `9.81` m/s² | Gravity magnitude |
| `STATIC_FRICTION` / `DYNAMIC_FRICTION` | `1.0` / `0.8` | Ground and target friction coefficients |
| `RESTITUTION` | `0.0` | Ground and target restitution |
