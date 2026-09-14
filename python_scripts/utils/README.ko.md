# 유틸리티

[English](README.md) · [루트로 돌아가기](../../README.ko.md)

Livestream 예제를 시작하고 네트워크 설정을 진단할 때 사용하는 공용 도구입니다.

## `live_stream.py`

Headless `SimulationApp`을 생성하고 `omni.kit.livestream.app`을 활성화하며 1280 × 720 Real-Time Path Tracing 스트림을 설정합니다. 대부분의 다른 Isaac Sim 모듈보다 먼저 가져와야 합니다.

```python
from utils.live_stream import simulation_app
```

이 모듈을 import하면 `SimulationApp`이 즉시 실행됩니다.

## `check_port.py`

환경 변수 중 `49100`, `47998`, `PUBLIC_IP`가 포함된 항목을 출력합니다.

```bash
<ISAAC_SIM>/python.sh python_scripts/utils/check_port.py
```

파일 이름과 달리 이 스크립트는 실제 리스닝 소켓을 조회하지 않습니다. Livestream 관련 포트 또는 공인 IP 값이 프로세스 환경에 설정되어 있는지 확인하는 용도입니다.

Linux에서 실제 TCP/UDP 리스닝 소켓을 계속 감시하려면 다음 명령을 사용합니다.

```bash
watch -d -n 0.1 "ss -lntup | grep -E '49100|47998'"
```

- `watch -n 0.1`: 0.1초마다 명령을 다시 실행합니다.
- `-d`: 이전 결과와 달라진 부분을 강조합니다.
- `ss -lntup`: 리스닝 TCP 소켓과 UDP 소켓을 나열하고, 권한이 있으면 프로세스 정보도 표시합니다.
- `grep -E`: `49100` 또는 `47998` 포트가 포함된 항목만 남깁니다.

출력이 없다면 현재 `ss` 결과에 두 포트가 나타나지 않은 것입니다. 프로세스 정보를 보려면 추가 권한이 필요할 수 있으며 Linux 환경에 `watch`와 `ss`가 설치되어 있어야 합니다.
