# Path Planning Core — 선택적 재계획(Selective Replan) 기반 MAPF

물류 창고 같은 환경에서, 동적 장애물이 실행 도중 나타났을 때 다중 로봇
경로(MAPF, Multi-Agent Pathfinding)를 **전체 재계획(Full Replan)** 대신
**선택적 재계획(Selective Replan) + 계층적 확장(Tiered Escalation)**으로
더 빠르고 성공률 높게 다시 계획하는 C++ 코어와, 그 결과를 눈으로 비교할
수 있는 Python GUI 데모.

핵심 아이디어를 한 문장으로: *"장애물이 생겨도, 그 장애물과 실제로
시공간적으로 마주칠 로봇만 다시 계획하고, 그래도 막히면 원인이 된 로봇만
추가로 끌어들여 범위를 넓힌다."* 자세한 설계 배경과 선행 연구와의 차별점은
[`DESIGN.md`](DESIGN.md)에, 단계별 구현 설명은 [`docs/`](docs/) 폴더에
정리되어 있다.

---

## 폴더 구조

```
Path_Planning_Core_/
├── core/                    C++ 알고리즘 코어 (프레임워크 비의존)
│   ├── include/mapf/        공개 헤더
│   │   ├── map.hpp           Cell, SpaceTimeCell, Map
│   │   ├── agent.hpp         Agent
│   │   ├── reservation_table.hpp  (x,y,t) 예약 장부
│   │   ├── space_time_astar.hpp   저수준 탐색(로봇 1대)
│   │   └── pbs.hpp           PBS(plan/replan/full_replan) — 고수준 조정 + 재계획 전략
│   └── src/                 위 헤더들의 구현
│
├── bindings/                 C++ 코어를 파이썬으로 노출하는 pybind11 바인딩
│   ├── mapf_bindings.cpp     바인딩 코드(Cell/Agent/Map/PBS 등)
│   ├── mapf_py.pyi           타입 스텁(자동완성/타입체크용, 실제 구현은 .cpp)
│   └── CMakeLists.txt
│
├── python/                   파이썬 쪽 코드(전부 위 bindings가 만드는 mapf_py를 그대로 호출)
│   ├── smoke_test.py          바인딩이 제대로 동작하는지 GUI 없이 확인하는 스크립트
│   ├── _pathsetup.py          컴파일된 mapf_py.pyd/.so를 sys.path에 등록하는 헬퍼
│   └── gui/                   2D 데모 GUI (pygame)
│       ├── app.py              진입점 — 실행은 아래 "2D GUI 실행" 참고
│       ├── maps.py             벤치마크와 동일한 open/corridor 맵 생성
│       ├── scenario.py         로봇 무작위 배치(시작점/목적지)
│       ├── sim_clock.py        SPACE로 진행하는 정수 시간 클럭
│       ├── render.py           pygame 렌더링(격자/로봇/경로/좌우 비교 패널)
│       └── *.pyi               위 각 모듈의 타입 스텁
│
├── tools/                     C++ 벤치마크 도구(논문/보고서용 데이터 수집)
│   ├── benchmark.cpp           run_benchmark 실행 파일 — CSV 출력
│   ├── benchmark_maps.hpp      벤치마크용 32x32 맵 2종(open/corridor) 생성
│   ├── plot_benchmark.py        CSV -> matplotlib 그래프 4종 PNG
│   ├── results/                 실행 결과 CSV
│   └── plots/                   생성된 그래프 PNG
│
├── tests/                     GoogleTest 단위 테스트
├── docs/                      단계별 설계/구현 설명 문서(00~08)
├── DESIGN.md                  전체 연구/설계 로드맵(Phase 0~5), 선행 연구와의 관계
└── CMakeLists.txt             최상위 빌드 스크립트
```

---

## 요구 사항

- CMake 3.15 이상
- C++17을 지원하는 컴파일러(Windows: Visual Studio 2022 / MSVC, Linux: GCC 또는 Clang)
- Python 3.7 이상(pybind11/GUI를 쓸 경우) + 개발 헤더(`python3-dev` 등)
- (GUI를 쓸 경우) `pygame` — `pip install pygame`

GoogleTest와 pybind11은 별도로 설치할 필요 없이 CMake `FetchContent`가 빌드
시점에 자동으로 받아온다.

---

## 빌드하기

```bash
# 1. CMake 구성(최초 1회, 또는 CMakeLists.txt를 수정했을 때)
cmake -S . -B build

# 2. 전체 빌드(테스트, 벤치마크, 파이썬 바인딩 전부)
cmake --build build --config Release
```

특정 타겟만 빌드하고 싶으면:

```bash
cmake --build build --target run_tests --config Release       # 단위 테스트
cmake --build build --target run_benchmark --config Release   # 벤치마크
cmake --build build --target mapf_py --config Release         # 파이썬 바인딩(GUI에 필요)
```

Windows(멀티 컨피그 MSBuild)와 Linux(단일 컨피그 Makefile/Ninja) 모두 위
명령이 그대로 동작한다. 다만 Windows는 빌드 산출물이 `build/<타겟>/Release/`
아래에, Linux는 보통 `build/<타겟>/` 바로 아래에 생긴다.

---

## 단위 테스트 실행

```bash
# Windows
./build/tests/Release/run_tests.exe

# Linux
./build/tests/run_tests
```

---

## C++ 벤치마크 실행

`full_replan`(전체 재계획) vs `replan`(선택적 재계획)의 성공률/실행시간을
맵 2종(open/corridor) x 로봇 수(5/10/20/40) x 50회 반복으로 비교해서 CSV로
출력한다.

```bash
# Windows
./build/tools/Release/run_benchmark.exe > tools/results/benchmark_5_10_20_40.csv

# Linux
./build/tools/run_benchmark > tools/results/benchmark_5_10_20_40.csv
```

CSV를 그래프로 시각화하려면(matplotlib, pandas 필요: `pip install matplotlib pandas`):

```bash
python tools/plot_benchmark.py tools/results/benchmark_5_10_20_40.csv tools/plots
```

`tools/plots/`에 4종 PNG가 생성된다:

| 파일 | 내용 |
|---|---|
| `success_rate.png` | `full_replan` / `selective_replan`(Tier 0-1만) / `selective_replan`(안전망 포함 전체) 성공률 비교 |
| `runtime_box.png` | 로봇 수별 실행시간(ms) 분포 박스플롯(로그 스케일) |
| `escalation_tier.png` | 선택적 재계획이 Tier 0 / Tier 1+ / 안전망 중 어디서 끝났는지 비율 |
| `plan_attempts.png` | 로봇 수가 늘수록 초기 배치 자체의 재시도 횟수(=난이도)가 어떻게 변하는지 |

---

## 2D GUI 실행

C++ 코어를 pybind11로 그대로 파이썬에 노출해서(`mapf_py` 모듈), 마우스로
장애물을 놓으면 `full_replan`과 `replan`을 **동시에** 계산해 좌우 화면에
나란히 비교해서 보여주는 인터랙티브 데모.

### 1) 먼저 `mapf_py` 바인딩을 빌드

```bash
cmake --build build --target mapf_py --config Release
```

### 2) (선택) 바인딩이 제대로 동작하는지 GUI 없이 먼저 확인

```bash
python python/smoke_test.py
```

`SMOKE TEST PASSED`가 뜨면 정상.

### 3) GUI 실행

리포 루트에서(패키지 모듈로 실행):

```bash
python python/gui/app.py --map open --agents 10 --seed 43
```

또는 `python/gui` 폴더 안에서 직접 실행해도 동일하게 동작한다:

```bash
cd python/gui
python app.py --map open --agents 10 --seed 43
```

### 실행 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--map` | `corridor` | 어떤 맵을 쓸지. `open`(3x3 기둥이 듬성듬성 배치된 넓은 통로) 또는 `corridor`(선반처럼 촘촘히 배치되어 통로 폭이 1칸뿐인 좁은 맵). 두 맵 모두 32x32 크기이며 `tools/benchmark_maps.hpp`와 동일한 레이아웃. |
| `--agents` | `8` | 로봇(에이전트) 수. 맵의 빈 칸 중에서 서로 겹치지 않게 시작점/목적지가 무작위로 배정된다. |
| `--seed` | `None`(미지정) | 무작위 배치를 재현 가능하게 고정하는 값. 같은 `--seed`를 주면 항상 같은 로봇 배치가 나온다(디버깅이나 발표용으로 같은 장면을 재현하고 싶을 때 사용). 생략하면 실행할 때마다 매번 다른 배치. |

### 조작법

| 조작 | 동작 |
|---|---|
| **마우스 좌클릭** | 클릭한 칸에 장애물을 놓고, `full_replan`(왼쪽 패널)과 `replan`(오른쪽 패널)을 동시에 계산해서 비교 표시. 벽이거나 현재 로봇이 서 있는 칸은 클릭이 무시된다. |
| **SPACE** | 시뮬레이션 시간을 한 스텝 진행(로봇이 자기 경로를 따라 한 칸 이동). |
| **R** | 완전히 새로운 무작위 시나리오로 재시작(시간/장애물/로봇 배치 전부 초기화). 재계획이 실패해서 화면이 잠긴 상태에서도 R키는 항상 동작한다. |
| **ESC** | 종료. |

각 패널 하단에는 소요 시간(ms), 성공/실패, `escalation_tier`(선택적
재계획이 Tier 0에서 끝났는지, 원인 로봇을 추가로 끌어들여 확장했는지,
안전망까지 갔는지), 전체 경로 길이가 표시된다. 재계획이 실패하면 그 즉시
로봇들이 현재 위치에 멈추고 화면 하단에 "R키를 눌러 재시작하라"는 안내가
뜨며, 새로 시작하기 전까지는 클릭/SPACE 조작이 막힌다.

---

## 알고리즘 요약

| 계층 | 역할 | 구현 |
|---|---|---|
| 저수준 탐색 | 로봇 1대의 최단 경로(시공간 A*) | `SpaceTimeAStar` |
| 고수준 조정 | 여러 로봇이 서로 충돌 없이 계획되도록 우선순위 순서로 조정 | `PBS::plan` |
| 재계획 전략(연구 기여) | 장애물이 생겼을 때 *누구를, 언제, 얼마나* 다시 계산할지 결정 | `PBS::replan`(Tier 0 → Tier 1+ → 안전망) |

- **`PBS::plan`**: 로봇들을 우선순위(리스트 순서) 순으로 하나씩 계획. 먼저
  계획된 로봇의 경로는 절대 바뀌지 않는다.
- **`PBS::full_replan`**: 장애물이 생기면 현재 위치를 새 출발점 삼아
  **전체** 로봇을 처음부터 다시 계획(베이스라인).
- **`PBS::replan`**: 장애물과 실제로 마주치는 로봇만(Tier 0) 먼저 다시
  계획하고, 그걸로 안 풀리면 막은 원인 로봇을 추적해서 재계획 대상에
  추가(Tier 1+), 그래도 안 되면 `full_replan`으로 안전하게 폴백.

자세한 원리는 [`docs/05_pbs.md`](docs/05_pbs.md), [`docs/06_selective_replan.md`](docs/06_selective_replan.md),
연구 포지셔닝은 [`docs/07_research_positioning.md`](docs/07_research_positioning.md)를 참고.

---

## 로드맵

- [x] Phase 1: C++ 코어(Map, Agent, SpaceTimeAStar, PBS)
- [x] Phase 2: pybind11 + pygame 2D GUI 데모
- [x] Phase 3: 선택적 재계획 + 계층적 확장, C++ 벤치마크·시각화
- [ ] Phase 4: Ubuntu + ROS2 + Gazebo 3D 시뮬레이션(TurtleBot3, 창고형 월드)
- [ ] Phase 5: 정량 평가 정리(성공률, makespan, sum-of-costs, 재계획 시간)

Phase 4/5의 상세 계획은 [`DESIGN.md`](DESIGN.md) §5, §6, §7 참고.
