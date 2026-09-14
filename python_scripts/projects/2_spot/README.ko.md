# Spot 이동

[English](README.md) · [루트로 돌아가기](../../../README.ko.md)

Isaac Sim의 `RobotPolicyRunner`를 사용하는 정책 기반 Spot 이동 예제입니다.

## 자동 경로

```powershell
& "<ISAAC_SIM>\python.bat" python_scripts/projects/2_spot/spot_standalone_livestream.py
```

Spot이 미리 정의된 속도 명령을 수행하며 명령 경로와 실제 이동 경로를 표시합니다.

## 키보드 제어

```powershell
& "<ISAAC_SIM>\python.bat" python_scripts/projects/2_spot/spot_standalone_keyboard.py
```

| 키 | 동작 |
| --- | --- |
| `W` / `S` | 전진 / 후진 |
| `A` / `D` | 좌 / 우 이동 |
| `Q` / `E` | 반시계 / 시계 방향 회전 |

두 스크립트 모두 `--device {cpu,cuda}`, `--engine {physx,newton}`, `--test` 옵션을 받으며 `python_scripts/utils/live_stream.py`의 공용 설정을 사용합니다. 경로 시각화 도우미는 `command_path.py`에 있습니다.
