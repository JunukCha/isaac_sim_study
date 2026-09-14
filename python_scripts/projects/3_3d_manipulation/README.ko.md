# 3D 매니퓰레이션

[English](README.md) · [루트로 돌아가기](../../../README.ko.md)

Lula 역기구학 해석기를 사용하는 Franka Panda 매니퓰레이션 실험입니다.

## 실행

```powershell
& "<ISAAC_SIM>\python.bat" python_scripts/projects/3_3d_manipulation/1_move_franka.py
```

주요 예제는 Franka end effector와 gripper를 다음 순서로 제어합니다.

```text
PRE_GRASP -> DESCEND -> CLOSE -> LIFT -> DONE
```

현재 타깃은 rigid-body 부착이 없는 시각적 오브젝트이므로 gripper를 닫아도 함께 이동하지 않습니다. `1_test_code.py`는 비교와 디버깅을 위해 남겨둔 초기 실험 버전입니다.
