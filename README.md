# Isaac Sim Study

[한국어](README.ko.md)

A collection of hands-on NVIDIA Isaac Sim exercises covering scene creation, robot locomotion, pick-and-place, and inverse-kinematics-based manipulation.

## Projects

| Project | Description |
| --- | --- |
| [0_tutorial](python_scripts/projects/0_tutorial/) | Basic scenes, objects, and Franka examples |
| [1_pick_place](python_scripts/projects/1_pick_place/) | Pick-and-place with Franka or UR10 |
| [2_spot](python_scripts/projects/2_spot/) | Policy-driven Spot locomotion |
| [3_3d_manipulation](python_scripts/projects/3_3d_manipulation/) | Lula IK Franka motion and physics-based grasp, lift, and release experiments |

## Requirements

- NVIDIA Isaac Sim with Assets access
- A compatible NVIDIA GPU and driver

Run standalone examples with the Python interpreter bundled with Isaac Sim:

```powershell
# Windows
& "<ISAAC_SIM>\python.bat" <script-path>
```

```bash
# Linux
<ISAAC_SIM>/python.sh <script-path>
```

See each project directory for detailed instructions. Reference NVIDIA samples are kept in [`python_scripts/orig_code`](python_scripts/orig_code/), and shared utilities are documented in [`python_scripts/utils`](python_scripts/utils/).

## Related Blog

Additional study notes and walkthroughs are available at [Grow Up by Coding](https://grow-up-by-coding.tistory.com/).

## License

No repository-wide license is currently declared. Some files contain NVIDIA Apache License 2.0 notices; check individual file headers before reuse.
