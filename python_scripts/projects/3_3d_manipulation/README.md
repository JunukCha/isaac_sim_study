# 3D Manipulation

[한국어](README.ko.md) · [Back to root](../../../README.md)

Franka Panda manipulation experiments using the Lula inverse kinematics solver.

## Run

```powershell
& "<ISAAC_SIM>\python.bat" python_scripts/projects/3_3d_manipulation/1_move_franka.py
```

The main example controls the Franka end effector and gripper through:

```text
PRE_GRASP -> DESCEND -> CLOSE -> LIFT -> DONE
```

The target is currently a visual object without rigid-body attachment, so it does not move with the closed gripper. `1_test_code.py` is an earlier experimental version retained for comparison and debugging.
