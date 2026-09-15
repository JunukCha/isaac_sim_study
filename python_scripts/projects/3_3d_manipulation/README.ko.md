# 3D 매니퓰레이션

[English](README.md) · [루트로 돌아가기](../../../README.ko.md)

Lula 역기구학 해석기를 사용하는 Franka Panda 매니퓰레이션 실험입니다. 말단 장치 이동부터 물리 기반 물체 파지와 놓기까지 다룹니다.

## 예제

| 스크립트 | 용도 |
| --- | --- |
| [`1_move_franka.py`](1_move_franka.py) | 시각적 타깃을 사용하는 기본 IK 이동 및 그리퍼 제어 |
| [`1_test_code.py`](1_test_code.py) | 비교와 디버깅을 위해 남겨둔 초기 실험 버전 |
| [`2_grasp_object.py`](2_grasp_object.py) | 위에서 접근해 그리퍼를 닫고 강체 타깃을 들어 올리는 실험; 완료 후 닫힌 상태 유지 |
| [`2_test_code.py`](2_test_code.py) | 파지·상승 완료 후 그리퍼를 열어 타깃을 놓는 실험 |

`1_move_franka.py`의 타깃은 강체 물리가 없는 시각적 오브젝트이므로 그리퍼를 닫아도 로봇과 함께 이동하지 않습니다.

## 실행

저장소 루트에서 Isaac Sim에 포함된 Python으로 스크립트를 하나씩 실행합니다.

```powershell
# Windows: 실행할 예제 선택
& "<ISAAC_SIM>\python.bat" python_scripts/projects/3_3d_manipulation/1_move_franka.py
& "<ISAAC_SIM>\python.bat" python_scripts/projects/3_3d_manipulation/2_grasp_object.py
& "<ISAAC_SIM>\python.bat" python_scripts/projects/3_3d_manipulation/2_test_code.py
```

```bash
# Linux: 파지 예제; 다른 예제는 파일명을 바꿔 실행
<ISAAC_SIM>/python.sh python_scripts/projects/3_3d_manipulation/2_grasp_object.py
```

예제는 공용 `utils.live_stream` 모듈로 headless Isaac Sim Livestream 세션을 시작합니다. 시작 방식과 네트워크 진단은 [유틸리티 가이드](../../utils/README.ko.md)를 참고하세요. Franka를 불러오려면 Isaac Sim Assets에 접근할 수 있어야 합니다.

## 파지와 놓기 동작

두 `2_*.py` 스크립트는 말단 장치와 그리퍼를 다음 순서로 제어합니다.

```text
PRE_GRASP -> DESCEND -> CLOSE -> LIFT -> DONE
```

고정된 하향 말단 자세, 바닥과 타깃의 충돌, 타깃의 강체 물리와 질량, 중력, 바닥과 타깃의 마찰 재질을 추가한 예제입니다. 파지·상승 위치는 고정된 타깃 기준 위치로 계산하므로 물체가 올라가도 상승 목표가 함께 높아지지 않습니다.

`DONE`에서 `2_grasp_object.py`는 그리퍼를 닫은 채 상승 자세를 유지합니다. `2_test_code.py`는 같은 자세에서 `open_gripper()`를 호출해 물체를 놓으며, 별도의 배치 위치로 이동하지는 않습니다.

상태 전환은 파지 성공 감지가 아니라 말단 위치와 그리퍼 닫힘 대기 시간을 기준으로 합니다. 시뮬레이션과 주기적으로 출력되는 로그(`State`, `EE distance`, `Target z`, `IK`)로 실제로 물체가 올라갔는지 확인하세요. 이 설명은 코드 기준이며, 이번 문서 수정 과정에서 실제 물리 파지 성공 여부를 검증하지는 않았습니다.

## 주요 파지 파라미터

실행할 `2_*.py` 스크립트 상단의 상수를 수정합니다. 두 스크립트의 현재 기본값은 같습니다.

| 상수 | 기본값 | 의미 |
| --- | --- | --- |
| `TARGET_SIZE` | `[0.05, 0.05, 0.05]` m | 약 0.08 m의 그리퍼 벌림 폭보다 작은 타깃 크기 |
| `TARGET_MASS` | `0.10` kg | 타깃 질량 |
| `PRE_GRASP_HEIGHT` | `0.20` m | 타깃 기준 위치 위 접근 높이 |
| `GRASP_HEIGHT` | `0.02` m | 타깃 기준 위치 위 파지 높이 |
| `LIFT_HEIGHT` | `0.25` m | 파지 위치에서 추가로 들어 올리는 거리 |
| `POSITION_THRESHOLD` | `0.02` m | 이동 상태 전환에 사용하는 위치 허용 오차 |
| `GRIPPER_CLOSE_DELAY` | `1.0` s | 닫힘 명령 후 상승 전 대기 시간 |
| `GRASP_ORIENTATION` | Euler `[0, pi, 0]`을 쿼터니언으로 변환 | 고정된 하향 자세 |
| `GRAVITY_MAGNITUDE` | `9.81` m/s² | 중력 크기 |
| `STATIC_FRICTION` / `DYNAMIC_FRICTION` | `1.0` / `0.8` | 바닥과 타깃의 정지·운동 마찰 계수 |
| `RESTITUTION` | `0.0` | 바닥과 타깃의 반발 계수 |
