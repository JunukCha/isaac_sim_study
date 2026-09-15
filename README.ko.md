# Isaac Sim Study

[English](README.md)

NVIDIA Isaac Sim으로 장면 생성, 로봇 이동, pick-and-place, 역기구학 기반 매니퓰레이션을 실습하는 학습용 저장소입니다.

## 프로젝트

| 프로젝트 | 내용 |
| --- | --- |
| [0_tutorial](python_scripts/projects/0_tutorial/README.ko.md) | 기본 장면, 오브젝트 및 Franka 예제 |
| [1_pick_place](python_scripts/projects/1_pick_place/README.ko.md) | Franka 또는 UR10 pick-and-place |
| [2_spot](python_scripts/projects/2_spot/README.ko.md) | 정책 기반 Spot 이동 |
| [3_3d_manipulation](python_scripts/projects/3_3d_manipulation/README.ko.md) | Lula IK 기반 Franka 이동과 물리 기반 파지·들어 올리기·놓기 실험 |

## 요구 사항

- Isaac Sim Assets에 접근할 수 있는 NVIDIA Isaac Sim 환경
- 호환되는 NVIDIA GPU 및 드라이버

독립 실행 예제는 Isaac Sim에 포함된 Python 인터프리터로 실행합니다.

```powershell
# Windows
& "<ISAAC_SIM>\python.bat" <스크립트-경로>
```

```bash
# Linux
<ISAAC_SIM>/python.sh <스크립트-경로>
```

자세한 설명과 실행 방법은 각 프로젝트 폴더를 참고하세요. NVIDIA 원본 예제는 [`python_scripts/orig_code`](python_scripts/orig_code/), 공용 유틸리티와 포트 확인 방법은 [`python_scripts/utils`](python_scripts/utils/README.ko.md)에 정리되어 있습니다.

## 관련 블로그

추가 학습 기록과 실습 내용은 [Grow Up by Coding](https://grow-up-by-coding.tistory.com/)에서 확인할 수 있습니다.

## 라이선스

저장소 전체에 적용되는 라이선스는 현재 명시되어 있지 않습니다. 일부 파일에는 NVIDIA Apache License 2.0 고지가 있으므로 재사용 전에 각 파일의 헤더를 확인하세요.
