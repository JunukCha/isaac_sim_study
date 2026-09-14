# Spot Locomotion

[한국어](README.ko.md) · [Back to root](../../../README.md)

Policy-driven Spot locomotion examples using Isaac Sim's `RobotPolicyRunner`.

## Scripted course

```powershell
& "<ISAAC_SIM>\python.bat" python_scripts/projects/2_spot/spot_standalone_livestream.py
```

Spot follows predefined velocity commands while the commanded and traveled paths are visualized.

## Keyboard control

```powershell
& "<ISAAC_SIM>\python.bat" python_scripts/projects/2_spot/spot_standalone_keyboard.py
```

| Key | Action |
| --- | --- |
| `W` / `S` | Forward / backward |
| `A` / `D` | Strafe left / right |
| `Q` / `E` | Rotate counterclockwise / clockwise |

Both scripts accept `--device {cpu,cuda}`, `--engine {physx,newton}`, and `--test`. They use the shared livestream setup in `python_scripts/utils/live_stream.py`. Route visualization helpers live in `command_path.py`.
