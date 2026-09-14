# Pick and Place

[English](README.md) · [루트로 돌아가기](../../../README.ko.md)

Franka 또는 UR10으로 큐브를 목표 위치까지 옮깁니다. cuMotion/RMPflow 팔 제어기와 로봇의 gripper 제어기를 조합한 예제입니다.

## 실행

```powershell
& "<ISAAC_SIM>\python.bat" python_scripts/projects/1_pick_place/pick_place_livestream.py --robot franka
```

옵션:

- `--robot {franka,ur10}`: 로봇 선택(기본값: `franka`)
- `--test`, `--test-steps N`: 유한한 테스트 실행
- `--headless`: headless 실행 요청
- `--usd-path PATH`: 로봇 USD 경로 재정의
- `--robot-config-dir DIR`: 사용자 `URDF`, `XRDF`, `rmp_flow.yaml` 사용

스크립트는 Livestream이 활성화된 `SimulationApp`을 시작하며, 작업이 끝난 뒤에도 애플리케이션을 닫을 때까지 스트리밍을 유지합니다.
