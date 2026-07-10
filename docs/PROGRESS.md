# 구현 진행 이력

이 문서는 `Path_Planning_Core_`(GitHub: `lch-98/mapf-selective-replan`) 리포의
단계별 구현 이력, 발견·수정한 버그, 설계 결정 사유를 시간순으로 기록한다.
Windows/Ubuntu 듀얼부팅 환경에서 작업하며, OS를 바꿔도(즉 로컬 AI 세션의
메모리가 없어도) `git pull`만으로 이 맥락을 그대로 이어받을 수 있도록
리포 안에 문서로 남긴다.

표준 작업 절차: 설계 확인 → 헤더 작성 → 구현 → 테스트 → 빌드+실행 →
결과 해석, 각 단계 완료 후 `run_tests` 통과 확인 후 커밋.

---

## 구현 순서 및 현재 상태 (docs/00_implementation_plan.md 0.2절 기준)

| 단계 | 내용 | 상태 |
|---|---|---|
| 0 | 빌드 시스템 뼈대 (CMake + GoogleTest) | 완료 |
| 1 | Cell, SpaceTimeCell, Map, Agent | 완료 |
| 2 | ReservationTable | 완료 |
| 3 | SpaceTimeAStar | 완료 |
| 4 | PBS::plan | 완료 (버그 2개 발견 및 수정) |
| 5 | PBS::replan (Tier 0→1→2, 06장) | 완료 (2026-06-28) |
| Phase 2 | pybind11 + pygame 2D GUI 데모 | 완료 (2026-07-10) |
| Phase 3 | 선택적 재계획 + 계층적 확장, C++ 벤치마크·시각화 | 완료 |
| Phase 4 | Ubuntu + ROS2 + Gazebo 3D 시뮬레이션(TurtleBot3, 창고형 월드) | 예정 |
| Phase 5 | 정량 평가 정리(성공률, makespan, sum-of-costs, 재계획 시간) | 예정 |

Phase 4/5의 상세 계획은 [`DESIGN.md`](../DESIGN.md) §5, §6, §7 참고.

---

## 1~4단계 코드 리뷰 (2026-06-28)

로컬 max-effort 리뷰(10개 각도 탐지 + 1-vote 검증)로 가장 심각한 버그 1건을
발견·수정.

**버그: vertex 등록이 무조건 덮어쓰기였음.** 4단계에서 고친 버그 1, 2는
`register_path`의 Tail 루프만 `reserve_if_unowned`로 바꿨는데, 그 앞의 vertex
등록 루프는 여전히 `reserve()`(무조건 덮어쓰기)를 쓰고 있었다. Space-time A*는
시작 칸의 점유를 검사하지 않으므로(다음 칸 이동 시에만 확인), 낮은 순위
로봇의 출발 칸이 더 높은 순위 로봇이 이미 정당하게 차지한 (칸,시각)과 같으면
(특히 start==goal일 때) 그 충돌이 검사 없이 사라지고 `PBS::plan()`이 실제로
충돌하는 두 경로를 성공으로 반환했다. → vertex 등록도 `reserve_if_unowned`로
바꾸고, 거절되면 즉시 `false` 반환하도록 수정. 회귀 테스트
`PBSTest.RejectsAgentWhoseStartCellIsAlreadyOwnedAtThatTime` 추가.

### 입력 검증 4건 (2026-06-29)

검증 실패는 `std::invalid_argument` 예외로 알리기로 결정(호출자가 즉시 잘못을
알 수 있도록).

- `Map(rows)`: 행마다 길이가 다르면 예외.
- `Map(width,height)`: 음수면 예외(생성자 본문 진입 전 멤버 초기화 리스트에서
  `std::vector` 생성이 먼저 일어나므로, `width<0?0:width`로 미리 걸러 crash 방지).
- `PBS::plan()`: agents에 중복 id 있으면 예외.
- `agents`가 빈 벡터면 빈 `PBSResult`(성공)로 처리.

---

## 4단계(PBS::plan) 버그 2건 — 사용자가 직접 코드 대조로 발견

1. **Tail Reservation이 다른 로봇의 vertex를 덮어쓰는 버그**:
   `register_path`의 Tail 루프(`reserve` 사용)가 무조건 덮어쓰기라서, 더 높은
   순위 로봇이 이미 정당하게 등록한 (칸,시각)을 낮은 순위 로봇의 Tail이
   지워버릴 수 있었다. → `ReservationTable::reserve_if_unowned(x,y,t,agent_id)`
   추가(이미 다른 agent_id가 점유한 칸은 거절, 비어있거나 자기 것이면 성공).
2. **Tail Reservation 거절이 조용히 무시되던 버그**: 위 함수가 `false`를
   반환해도 `register_path`가 그 반환값을 버리고 계속 진행해서, 모순된
   상태를 성공으로 잘못 보고했다. → `register_path`가 `bool`을 반환하도록
   바꾸고, 한 번이라도 거절되면 `plan()`이 즉시 `nullopt`로 실패 처리.

## 설계 변경 — "목적지 임시 선점" 완전히 제거

초기 구현(05장 5.6절 원안)은 `plan()` 시작 시 모든 로봇(아직 차례 안 온 로봇
포함)의 목적지를 t=0~max_timestep까지 미리 통째로 선점했다. 충돌은 "같은
칸+같은 시각"에서만 생기므로 미리 선점은 불필요하게 보수적이었고, 오히려
다른 로봇의 Tail Reservation을 침범하는 버그의 원인이기도 했다. → 이 장치를
완전히 삭제, Tail Reservation 하나만으로 충분함을 `docs/05_pbs.md` 5.6절에
반영. `unreserve_if_owned_by`는 이제 `PBS`에서 호출되지 않지만
`ReservationTable`의 일반 기능으로 남아있다.

## PBS의 알려진 한계

PBS는 "낮은 순위가 항상 양보"하는 고정 우선순위 구조라서, 1순위 로봇이
탐색할 때는 항상 테이블이 비어있는 상태에서 최단경로를 그냥 가져간다 —
낮은 순위를 위해 우회하는 일이 없다. "사실 한쪽이 조금만 비켜주면 둘 다
풀리는" 상황에서도 PBS 자체로는 실패할 수 있다. 이것이 `docs/06_selective_replan.md`
6.5절 "Tier 0의 한계"이고, Tier 1(계층적 확장)의 출발점이다.

---

## 5단계(PBS::replan) 구현 (2026-06-28)

`core/include/mapf/pbs.hpp`/`core/src/pbs.cpp`에 `PBS::replan()`,
`ReplanResult`, `PBSConfig` 추가. `docs/06_selective_replan.md`의
Tier 0(영향받은 로봇만) → Tier 1..N(`get_owner`로 원인 로봇 추적,
working_ids 확장) → 안전망(전체 재계획, 현재 위치 기준) 흐름을 구현.

### 구현 중 발견한 설계 빈틈 — "barrier 등록 자체가 거절되는" 경우

06장 원문은 Tier 1 추적을 "A* 탐색이 `blocked_attempts`를 남기고 실패했을 때
`get_owner`로 추적"이라고만 설명한다. 그런데 working_ids에 없는 로봇(K)을
기존 경로로 barrier 등록하는 단계 자체가 `register_path` 내부에서 거절되는
별개의 실패 경로가 있었다 — 이때는 `blocked_attempts`가 전혀 채워지지 않아
(A* 탐색 자체는 성공했으므로) 기존 메커니즘으로는 Tier 1이 트리거되지 않고
곧장 안전망으로 빠졌다.

해결: `try_replan_set`에 `out_blocked_owner` 채널을 추가. barrier 등록이
거절되면 "거절당한 로봇(K) 자신"을 `replan()`에 알려주고, `replan()`이 그
로봇을 working_ids에 추가한다 — 사실상 "Tier 1의 두 번째 입구". 이 케이스는
`docs/06_selective_replan.md` 6.6절 끝에 문서화되어 있다.

### 테스트 시나리오 설계에서 막혔던 함정

1. K가 working_ids에 있을 때 R보다 우선순위가 높으면 Tier 1로 절대 못 구한다
   (먼저 계획된 로봇의 경로는 절대 안 바뀌므로).
2. K의 목적지 자체가 R의 우회 경로 위에 있으면 Tier 1로 절대 못 구한다
   (A*는 Tail Reservation을 모르고 최단경로만 찾으므로, Tail 충돌은 재탐색으로
   해소가 안 된다). Tier 1이 효과를 보는 건 충돌이 K의 목적지가 아니라
   지나가는 중간 vertex일 때뿐이다.
3. 위 두 함정 때문에 `previous_paths`를 `PBS::plan()` 결과에 의존하지 않고
   손으로 직접 구성하는 게 의도한 시공간 충돌을 정확히 만들 때 더 빠르고
   확실했다.

---

## C++ 벤치마크 도구(tools/) 구축 (2026-06-29)

DESIGN.md의 Phase 2/3 완료 기록은 옛 폴더(`Path_Planning_Core`, 언더스코어
없음)의 것이었고, 현재 폴더에는 애초에 `python/`이 없었다. 그래서 GUI보다
먼저 C++로 직접 데이터를 수집하기로 함 — pybind11 빌드 설정 없이 기존
CMake 구조에 바로 추가 가능해서.

**만든 것**: `tools/benchmark_maps.hpp`(32x32 맵 2종: `make_open_map()` 넓은
통로, `make_corridor_map()` 좁은 통로), `tools/benchmark.cpp`(`run_benchmark`
실행 파일, CSV 출력), `PBS::full_replan()`을 `private`에서 `public`으로 승격.

**비교 대상**: 방법A=`full_replan()`(베이스라인), 방법B=`replan()`(제안 기법).
같은 초기 경로·같은 장애물·같은 `current_time`을 두 방법에 동일하게 적용해
공정 비교.

### 발견·수정한 버그 3건 (모두 사용자가 직접 의심해서 발견)

1. **40대 제외 결정의 근거**: 16x16 완전히 빈 맵에서도 40대 `plan()` 성공률이
   0%임을 재현 확인 — 맵 문제가 아니라 PBS의 고정 우선순위 구조 자체가
   "로봇 수가 많을 때 무작위 배치"에 본질적으로 취약한 것. 이후(2026-07-09)
   버그 수정과 함께 다시 포함(`agent_counts = {5, 10, 20, 40}`) — 성공률이
   낮은 건 PBS의 실제 한계를 보여주는 데이터이므로 그대로 둔다.
2. **`current_time`을 "경로 길이의 절반"으로 임의로 정했던 설계 오류**: 장애물과
   무관한 시점이면 재계획이 사실상 장애물 없는 것과 같아져 비교가 무의미해진다.
   → 장애물이 실제로 어느 로봇의 경로와 충돌하는 시각을 찾아 그 시점을
   `current_time`으로 쓰도록 수정.
3. **충돌 시각 그대로 `current_time`을 쓰면 100% 실패하는 버그**: 장애물 칸의
   시각을 그대로 쓰면 "그 로봇이 지금 정확히 그 장애물 칸에 서 있다"는 뜻이
   되어, `register_path`가 시작 칸이 이미 장애물(owner=-1) 소유라며 거절한다.
   → `current_time`을 `chosen.t - 1`(도착 한 스텝 전)로 수정.
4. **초기 `plan()` 실패 시나리오는 재시도해서 제외**: 성공하는 시나리오가
   나올 때까지 재시도(최대 200회)하고, `plan_attempts` 컬럼으로 재시도
   횟수를 CSV에 남긴다.

### 결과 데이터 (초기, tools/results/benchmark_5_10_20.csv)

성공률: 모든 조합에서 selective_replan이 full_replan보다 높음(로봇 20대:
open 60%→84%, corridor 56%→84%). 속도도 항상 더 빠름. escalation_tier
분포에서 corridor(좁은 통로)가 open보다 Tier1 발동 비율이 높음(corridor,20:
19% vs open,20: 7%) — DESIGN.md가 예측한 가설과 일치.

---

## 벤치마크 2차 수정 — 설계 결함 2건 + 멀티스레드화 (2026-06-30, 커밋 980f589)

### 지적 1: "장애물이 실제로 기존 경로를 막는지 검증을 안 한다"

`make_obstacles_that_actually_block`이 "경로 중간 칸"에서 장애물을 고르긴
했지만, 실제로 막는다는 걸 직접 검증하지는 않고 추론만 했다. → `path_hits_obstacle`을
`private`에서 `public static`으로 승격(`replan()`이 실제로 쓰는 판별 함수를
벤치마크가 그대로 재사용) + `obstacles_actually_block_someone()` 추가해서
장애물을 고른 직후 직접 검증, 안 막으면 시나리오를 버리고 재시도.

(참고: `full_replan` 실패 시에만 `replan`을 시도하는 구조가 아니냐는 질문도
있었지만 이건 오해였다 — 항상 둘 다 독립 실행해서 같은 시나리오를 비교하는
게 맞는 설계다.)

### 지적 2(파생 발견): 멀티스레드화 → 극단적으로 느린 trial 발견

멀티스레드화(std::thread + 동적 작업 큐) 적용 후에도 wall-time이 거의
안 줄었다(CPU는 1058%인데 시간은 그대로) — CSV에서 `selective_replan_ms`
최댓값을 sort해보니 43731ms짜리 trial 발견. 원인: 장애물 후보를 고를 때
"그 칸 주인 로봇 자신의 goal"만 피하고 **다른 로봇의 goal과 겹치는지는
확인 안 함**. 그 칸이 다른 로봇 X의 목적지였던 경우 X는 영원히 도달 불가능한
상태가 되고, A*는 이를 모른 채 `max_timestep(256)`까지 헛탐색한다.

해결: `collect_path_cells`와 `make_obstacles_that_actually_block`이 모든
로봇의 goal 집합을 만들어, 장애물 후보가 그 어떤 goal과도 안 겹치는 칸만
고르도록 수정. 결과: 90~100초 → 24초(주로 버그 수정 효과).

멀티스레드 구현: task(`map x num_agents x trial`)마다 `base_seed + task_index`로
독립 시드 부여(재현성 보장), 결과는 `std::vector<std::string> results`에
task 순서대로 미리 자리를 만들어 채운 뒤 한 번에 출력(표준출력 줄 섞임 방지).
`tools/CMakeLists.txt`에 `find_package(Threads REQUIRED)` 추가(Ubuntu 이전 시
pthread 자동 처리).

---

## 벤치마크 데이터 사각지대 수정 + matplotlib 시각화 (2026-07-09~10)

`tools/benchmark.cpp`에서 사각지대 3개를 추가로 발견·수정:

1. **`collect_path_cells`가 다른 로봇의 start와 안 겹치는지 확인 안 함**:
   `all_goals`만 걸러내고 `all_starts`는 안 걸러내서, `chosen.t==1`인 후보가
   뽑히면 `current_time=0`이 되어 다른 로봇의 start(t=0)와 충돌하는 모순이
   생겼다. → `all_starts`도 필터링하도록 수정.
2. **fallback(`blocking_cell = free_cells.front()`)이 start/goal과 겹칠 수
   있음**: `path_cells`가 비어 `current_time=0`으로 고정되는 분기에서 같은
   문제 재현. → `all_starts`/`all_goals` 둘 다에 속하지 않는 첫 칸을 찾도록 수정.
3. **로봇 40대 벤치마크 데이터 없음**: 다시 포함(`agent_counts = {5, 10, 20, 40}`).
   corridor 40대는 `plan_attempts` 평균 187, 성공 표본이 50개 중 6개뿐인
   경우도 있지만, 이는 버그가 아니라 PBS 고정 우선순위 구조의 실제 한계.

**`tools/plot_benchmark.py`(신규)**: CSV → matplotlib 4종 그래프
(`success_rate.png`, `runtime_box.png`, `escalation_tier.png`,
`plan_attempts.png`). 결과는 `tools/results/benchmark_5_10_20_40.csv` +
`tools/plots/`.

- `success_rate.png`의 핵심 논점: `selective_replan_ok=1`에는 Tier 0/1 성공과
  `escalation_tier=-1`(안전망=`full_replan`을 그대로 호출) 성공이 섞여 있다.
  안전망 포함 전체는 이론상 항상 `full_replan` 이상이지만, **Tier 0/1만
  떼어보면 `full_replan`보다 낮게 나올 수 있다**(corridor+5대에서 47개 중
  1개가 안전망행) — Tier 0/1은 일부 로봇만 재탐색하고 나머지는 옛 경로를
  장벽으로 고정하는 "제한된 탐색"이라, 그 제한 안에서 답이 없는 경우가
  실제로 존재하기 때문이다. full_replan/전체(안전망 포함)/Tier0-1만 세
  막대로 나눠서 이 관계를 명시적으로 보여준다.
- 벤치마크 실행시간 타이머는 처음부터 `full_replan`/`replan` 호출 앞뒤로만
  고정되어 있었다(시나리오 탐색 시간은 미포함). 예전 CSV(107ms대)가 느렸던
  진짜 원인은 그 당시 "다른 로봇의 goal을 장애물로 잘못 고르는" 버그로 A*가
  `max_timestep=256`까지 헛탐색했기 때문 — 알고리즘 자체는 항상 빨랐다
  (3.5ms 안팎이 정상).

---

## Phase 2: pybind11 + pygame 2D GUI 데모 (2026-07-10)

DESIGN.md Phase 2 원안(pybind11+pygame)을 그대로 따름 — 순수 Python 재구현이
아니라 실제 컴파일된 C++ PBS/Map/Agent를 그대로 호출.

### 새 디렉터리
- `bindings/`: `CMakeLists.txt`, `mapf_bindings.cpp`(바인딩 코드 전부).
- `python/`: `_pathsetup.py`(빌드된 `.pyd`를 sys.path에 등록), `smoke_test.py`
  (바인딩 단독 검증), `gui/`(app.py, render.py, maps.py, scenario.py,
  sim_clock.py).

### CMake 통합
최상위 `CMakeLists.txt`에 `find_package(Python3 ...)` + pybind11 FetchContent
+ `add_subdirectory(bindings)` 추가. 계획과 실제가 달랐던 점 2가지:
1. pybind11 v2.13.6은 `std::optional` 지원이 `pybind11/stl/optional.h`(이
   버전엔 없음)가 아니라 `pybind11/stl.h`에 이미 통합되어 있다.
2. `py::self` 연산자(`__eq__`, `__lt__`)를 쓰려면 `pybind11/operators.h`가
   별도로 필요하다.

### 바인딩 범위 (`bindings/mapf_bindings.cpp`)
`Cell`, `SpaceTimeCell`, `Agent`, `Map`, `AStarConfig`, `PBSConfig`,
`ReplanResult`, `PBS`만 바인딩. `ReservationTable`/`SpaceTimeAStar`는 의도적으로
제외(GUI가 안 쓰고, 댕글링 참조 위험 회피). 핵심 포인트:
- `Cell`/`SpaceTimeCell`에 `__hash__`를 GUI 편의를 위해 새로 추가(C++ API에는
  없던 것).
- `PBSResult`는 커스텀 바인딩 없이 `pybind11/stl.h`의 제네릭 caster가
  `dict[int, list[SpaceTimeCell]]`로 자동 변환한다.
- **`py::keep_alive<1,2>()`가 PBS 생성자 바인딩에 필수** — PBS가 `const Map&`을
  멤버로 저장하므로, Python Map 객체가 PBS보다 먼저 GC되면 댕글링 참조 크래시.

### GUI 설계 결정
- 장애물 클릭 시 `full_replan`과 `replan`을 동시에 호출해 좌우 분할 화면으로
  비교(경로, ms, escalation_tier).
- 로봇 배치는 매번 무작위, 기본 맵 corridor.
- 시간(`current_time`)은 SPACE로 한 스텝씩 수동 진행.
- `current_time` 시맨틱: 벤치마크의 `chosen.t-1` 역산은 "합성 시나리오 전용"
  트릭이라 GUI엔 적용 안 됨 — GUI에서 `current_time`은 사용자가 SPACE로 직접
  통제하는 독립 변수. "클릭한 칸이 그 순간 로봇이 서 있는 칸과 겹치면 안
  된다"는 교훈만 `handle_click`에서 사전 검증으로 그대로 적용.
- pyd 검색: `python/_pathsetup.py`가 `build/bindings/Release|Debug`를 뒤져서
  `sys.path`에 등록. CMake는 `mapf_py.cp312-win_amd64.pyd`처럼 ABI 태그가
  붙은 이름으로 산출물을 만든다(import는 `mapf_py`로 정상 동작).

### 실행 중 발견·수정한 버그 4건

1. **`python\gui>python app.py` 직접 실행 시 `ModuleNotFoundError`**: 원래
   절대 임포트라 `-m python.gui.app`(리포 루트에서)로만 실행 가능했다.
   `python app.py`(폴더 안에서 직접) 방식도 지원하기 위해, `app.py` 상단에서
   `gui`/`python` 폴더를 `sys.path`에 직접 넣고 패키지 접두어 없는 임포트로
   전환 — 이제 두 실행 방식 다 지원.
2. **한글 텍스트 깨짐**: `SysFont("consolas", ...)`는 한글 글리프가 없음 →
   `malgungothic`(맑은고딕)으로 교체.
3. **창 아이콘이 pygame 기본 로고**: `render.make_app_icon()`을 추가해 외부
   이미지 없이 코드로 격자+로봇 경로 아이콘을 직접 그려서 적용.
4. **(가장 중요) FAILED인데 로봇이 장애물을 뚫고 지나가는 것처럼 보임**:
   `recompute()`가 재계획 실패 시 경로를 갱신하지 않고 그대로 둬서, 화면엔
   장애물 생기기 전의 낡은 경로가 계속 그려졌다. → `_frozen_paths_at_current_time()`
   추가: 실패 시 각 로봇을 "과거 이동 이력 그대로 + current_time부터는
   제자리에 정지"로 강제 표시.
5. **실패 상태에서도 계속 조작 가능**: `App.is_failed` 프로퍼티 추가(둘 중
   하나라도 실패면 True). 실패 상태에선 클릭/SPACE 다 무시하고 안내만 표시,
   **R키로 완전히 새로운 무작위 시나리오 재시작**(`_start_new_scenario()`).

### 검증 방법 (화면을 못 보는 서버 환경에서)
`SDL_VIDEODRIVER=dummy` 환경변수로 headless pygame 초기화 후 `App` 클래스를
코드로 직접 구동(클릭 좌표 계산, `recompute()` 호출 등)해서 로직을 검증하고,
`pygame.image.save(app.screen, ...)`로 렌더링 결과를 PNG로 저장해 실제로
눈으로 확인하는 방식을 사용 — 이 프로젝트에서 GUI 버그를 재현·검증하는
표준 절차.

---

## README 및 타입 스텁 정비 (2026-07-10)

Ubuntu 전환을 앞두고 `README.md`(사용법, 폴더 구조, CLI 인자 설명)와 `.pyi`
타입 스텁(GUI 모듈들의 공개 인터페이스 문서화)을 정비해, 코드베이스 자체가
온보딩 문서 역할을 하도록 강화. 이후(같은 날) 다음 항목도 보완:

- `run_benchmark`가 명령줄 인자를 받지 않는다는 점, `tools/plot_benchmark.py`의
  위치 인자(`csv_path` 필수, `output_dir` 선택) 설명 추가.
- 빌드 명령이 리포 루트 기준이라는 전제 명시.
- `run_tests`/`run_benchmark`/`mapf_py` 모두 CMake의 `target_link_libraries`로
  `mapf_core`를 링크하므로, 타겟 빌드 시 CMake가 `mapf_core`를 자동으로 먼저
  빌드한다는 점 명시(사전에 별도로 `mapf_core`를 미리 빌드해둘 필요 없음).

## Ubuntu로 개발 환경 전환 (2026-07-10)

다음 단계(Phase 4: Gazebo 3D)부터는 Ubuntu 부팅으로 넘어가서 GitHub에서 이
리포를 `git pull` 받아 진행한다. Windows 빌드 산출물
(`build/bindings/Release/mapf_py.cp312-win_amd64.pyd` 등)은 플랫폼
종속적이라 Ubuntu에서는 무의미하며, Ubuntu에서는 CMake 재구성+재빌드가
필요하다(`.gitignore`에 `build/`가 있어 애초에 커밋 안 됨).

### 다음에 할 일 — Phase 4

Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11로 3D 확장(`DESIGN.md` §5
Phase 4, §6 환경 구성 참고 — 듀얼부팅 노트북에 이미 설치된 조합을 그대로
사용). 같은 C++ 코어(`core/`)를 `rclcpp` 노드로 래핑(재구현 금지),
TurtleBot3 N대 네임스페이스 분리 배치, 창고형 월드, Gazebo actor로 동적
장애물 주입. 저수준 추종은 Nav2 전체보다 간단한 pure-pursuit로 시작(MAPF
레이어 평가가 목적이므로).

동적(움직이는) 장애물 지원으로 PBS 자체를 확장하는 별도 분석은
[`09_dynamic_obstacles_plan.md`](09_dynamic_obstacles_plan.md) 참고.
