# Pick and Place

[한국어](README.ko.md) · [Back to root](../../../README.md)

Moves a cube to a target position with either a Franka or UR10. The example combines a cuMotion/RMPflow arm controller with the robot's gripper controller.

## Run

```powershell
& "<ISAAC_SIM>\python.bat" python_scripts/projects/1_pick_place/pick_place_livestream.py --robot franka
```

Options:

- `--robot {franka,ur10}`: robot selection (default: `franka`)
- `--test` and `--test-steps N`: finite test execution
- `--headless`: request headless execution
- `--usd-path PATH`: override the robot USD
- `--robot-config-dir DIR`: use custom `URDF`, `XRDF`, and `rmp_flow.yaml`

The script starts a livestream-enabled `SimulationApp` and keeps the stream running after the task until the application is closed.
