# CLAUDE.md — 이 저장소에서 일할 때의 규칙

> Windows와 Ubuntu(듀얼부팅)를 오가며 작업한다. AI 세션의 메모리는 OS마다 따로라서, **지켜야 할 규칙과 현재 위치는 이 파일과
> `docs/PROGRESS.md`에 남긴다.** 새 세션은 먼저 이 파일 → `docs/PROGRESS.md`(맨 위 표와 "남은 작업 일정", 맨 아래 기록) →
> `docs/29_step6_easy_summary.md` 순서로 읽는다.

## 연구 한 줄
물류 창고에서 로봇들이 움직이는 도중 예기치 않은 장애물이 나타났을 때, 로봇별 경로만 받아서(처음 계획 방법과 무관),
**PP(고정 우선순위 계획) + 시공간 A\*** 엔진으로 영향받은 로봇만 빠르게 다시 계획하는 선택적 재계획. 주제·차별점은 `docs/26`.

## 용어 (헷갈리지 말 것)
- 엔진은 **PP**다. **PBS가 아니다**(진짜 PBS = Ma et al. 2019, 순서를 탐색하는 다른 방법). 코드 이름 `PrioritizedPlanner`.
- 우리 방법 = Tier 0(영향받은 로봇만) + ① 고정 먼저 + ② 도달 확인 + ③ 길 위 로봇 구조(Tier 1~2) + 안전망.
  - 안전망 ① 전체 재계획, 안전망 ② 전체 재계획까지 실패하면 처음 영향받은 로봇 순서를 바꿔 다시(최대 3번, 30장).
  - 벤치마크 옵션: `--order fixed-first --reachability --rescue path --tiers 2 --order-restarts 3` (모든 옵션 목록: `docs/OPTIONS.md`)
- 실행 순서: Tier 0 → Tier 1 → Tier 2 → 안전망 ①(전체 재계획) → 안전망 ②(처음 영향받은 로봇 순서만 바꿔 Tier 0~2를 다시, 최대 3번).
  (2026-11-20 전 우리 방법 실험도 ①②③은 켜져 있었고, 단계만 Tier 0 → Tier 1 → 안전망 ①이었다. 11-20에 Tier 2와 안전망 ②만 더함.
  "원래 방식"(legacy, 코드 기본값)은 ①②③이 모두 꺼진 Tier 0 → Tier 1 → 안전망 ①.)
- 엔진 고정 설정(논문 실험 조건에 반드시 적을 것): A\* 동점 규칙 = f 작은 쪽 → g 큰 쪽 → 먼저 넣은 쪽(`--tie-break larger-g`, 기본값),
  `max_timestep` 256(128×128은 512). 동점 규칙을 바꿔도 결론 유지(21장 21.8).
- "스스로 푼 수" = 안전망(①②) 없이 Tier 0~2에서 푼 수. 작업 시간(service time): 도착 후 작업 중인 로봇은 절대 안 움직임.
- 예전 문서의 "PIEF"는 잘못된 이름 → Švancara 2019의 **OID**.
- **칸 위치는 "행·열"로 말한다(사용자 요청 2026-11-27)**: 행 = 가로줄, 위에서 아래로 0행부터(코드의 `y`) / 열 = 세로줄, 왼쪽부터 0열(코드의 `x`).
  말할 때는 "9행 7열"처럼 **행을 먼저**. 코드·JSON의 좌표는 `[x, y]` = **[열, 행]** 순서라 거꾸로이므로, 숫자 쌍만 쓰지 말고 항상 "행·열"을 붙인다.

## 일하는 방식 (사용자 요청)
- **한국어**, 쉽고 자세하게. 채팅은 짧게, 자세한 설명·결과는 `docs/`에 md로 남긴다(쉬운 요약 + 자세한 장).
- 순서: **설계 → 사용자가 이해 → 코드 → 테스트 → 빌드 → 결과 해석**. 사용자가 이해 못 한 채로 다음 큰 단계로 넘어가지 않는다.
- 새 기능은 **스위치(기본 꺼짐)**로 넣고, 꺼짐일 때 기존 벤치마크 결과가 **시간 열(_ms) 빼고 칸마다 같은지** 확인한다.
  테스트가 처음부터 통과하면 코드를 일부러 틀리게 바꿔 테스트가 잡는지(변이 검사) 확인한다.
- 논리적 정확성이 우선. 결과를 부풀리지 말고, 안 되는 조건(예: 넓은 맵에선 차이 없음)도 그대로 적는다.
- 커밋은 한국어 메시지, 끝에 `Co-Authored-By` 줄. push는 사용자가 요청할 때만.
- 범위 결정: ML(학습) 아이디어는 제외. 3D·실제 로봇 실험은 **전체 재계획과만** 비교. Park식·LNS2식 비교는 2D에서만, LNS2식은 9번 직전(6″).

## 빌드·테스트
Windows(Visual Studio):
```
cmake --build build --config Release
build/tests/Release/run_tests.exe
python python/smoke_test.py
build/tools/Release/run_benchmark.exe [옵션]
```
Ubuntu:
```
sudo apt install build-essential cmake git python3-dev python3-pip
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/tests/run_tests
python3 python/smoke_test.py
./build/tools/run_benchmark [옵션]
```
(Ubuntu에서는 실행 파일 경로에 `Release/`가 없다. 파이썬 모듈은 `build/bindings/mapf_py*.so` — `python/_pathsetup.py`가 찾는다.)
3D(Ubuntu 전용, `ros2/README.md`): `cd ros2 && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash`,
`ros2 launch mapf_gazebo scenario.launch.py scenario:=corridor10_r6_s1`(화면은 `gui:=true`, 시나리오 JSON 형식은 31.19). **AI가 Gazebo를 띄우고 끌 때**: 먼저 `ps`로 사용자가 띄워 둔 Gazebo가 있는지 확인한다(있으면 같은 월드로 착각한 점검이 돌고, 새 gzserver는 죽는다 —
2026-11-26 실제로 겪음). 끌 때는 **자기가 띄운 launch의 PID로만** `kill -INT <PID>` 한다(띄울 때 `bash -c 'echo $$ > pidfile; exec ros2 launch ...'`로 PID를 적어 둠, 확인은 `pgrep -xa gzserver`). `pkill -f "ros2 launch mapf_gazebo"`는 사용자의 Gazebo까지 끄고,
패턴이 든 셸 자신도 끈다.

## 지금 위치 (2026-11-26)
일정 1~6 완료(6번 나머지 6″은 9번 직전). 다음은 **일정 7: ROS2 + Gazebo 3D** — `docs/31_3d_gazebo_design.md`(2026-11-25): 실행은 ADG,
재계획은 "지금 모습 = 새 0번" + 약속을 지키는 번호 다시 붙이기(31.6), 기본 6대. 31.13 확정(2026-11-26, 바닥 크기만 미정), 안전은 세 겹(31.4). **7-0·7-1·7-2 완료(31.18·31.20·31.21), 다음은 7-3**(작업 순서 7-0~7-6과 단계별 "끝났다" 기준 = **31.12**).
