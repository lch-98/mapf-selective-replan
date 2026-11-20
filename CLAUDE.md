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
  - 벤치마크 옵션: `--order fixed-first --reachability --rescue path --tiers 2 --order-restarts 3`
- "스스로 푼 수" = 안전망(①②) 없이 Tier 0~2에서 푼 수. 작업 시간(service time): 도착 후 작업 중인 로봇은 절대 안 움직임.
- 예전 문서의 "PIEF"는 잘못된 이름 → Švancara 2019의 **OID**.

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

## 지금 위치 (2026-11-20)
일정 1~6 완료(6번 나머지 6″은 9번 직전). 다음은 **일정 7: ROS2 + Gazebo 3D** — 시작 전에 `docs/31`(3D 설계)을 사용자와 정한다.
