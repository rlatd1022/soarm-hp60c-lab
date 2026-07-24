# soarm_lab — SO-ARM101 실습 제공물

MuJoCo 시뮬레이션과 실물 SO-ARM101 제어 실습용 패키지입니다.
학생은 안을 몰라도 됩니다. `arm` 하나로 시작하고, 필요할 때 열어 봅니다.

## 설치
```bash
pip install mujoco numpy      # 시뮬
pip install pyserial          # 실물(driver_sdk)까지 쓸 때
```

## 사용
작업 폴더에 이 `soarm_provided` 폴더를 두고, 스크립트 맨 위에서 경로를 얹습니다.
```python
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "soarm_provided"))

from soarm_lab import arm, SCENE

arm.live()                              # 라이브 뷰어 창
arm.go([0.25, 0.0, 0.15])               # 손끝을 그 좌표로 (시뮬)
arm.go([0.25, 0.0, 0.15], real=True)    # 같은 좌표를 실물로
arm.wait()
```

## 구성 (`soarm_lab/`)
| 파일 | 역할 |
|---|---|
| `arm.py` | 제어 진입점 — `arm.go` / `run` / `grip` / `grasp` / `ball_xy` / `live` / `wait` |
| `ik_core.py` | 역기구학 (DLS) — 좌표 → 관절각 |
| `fk_core.py` | 순기구학 (POE) — 관절각 → 위치 |
| `sim.py` | MuJoCo 시뮬 백엔드 |
| `real.py` | 실물 백엔드 (시리얼 직결) |
| `driver_sdk.py` | STS3215 서보 드라이버 |
| `grasp.py` | 파지 접근점 계산 (`approach_xy`) |
| `grasp_scene.py` | 파지 씬 빌더 (공 + 바구니 + 물리) |
| `models/` | 공식 SO-101 모델 (scene.xml + 메시) |

실행 예제는 `examples/pick_place.py` (좌표함수 pick & place).

모델 출처: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) — Simulation/SO101
