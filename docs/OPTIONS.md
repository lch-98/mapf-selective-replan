# 켜고 끌 수 있는 옵션 전체 목록

> 2026-11-20 정리. 코드의 **기본값은 "원래 방식"**이다(예전 테스트·결과가 그대로 유지되게). **우리 방법은 옵션을 켜야** 한다.
> 일정 10번(정리 리팩토링)에서 우리 방법을 기본값으로 바꾸고 원래 방식 코드를 지울 예정.

## 우리 방법 한 줄

```
run_benchmark --order fixed-first --reachability --rescue path --tiers 2 --order-restarts 3
```
파이썬(GUI `--method ours`, `tools/independent_check.py`)도 같은 설정이다.

---

## 1. 재계획 설정 — `ReplanConfig` (C++ `core/include/mapf/prioritized_planner.hpp`, 파이썬 `mapf_py.ReplanConfig`)

세 가지 설정을 구분한다:
- **원래 방식**(legacy) = 코드 기본값. ①②③이 모두 꺼짐. 처음 만든 선택적 재계획.
- **우리 방법(11-20 이전)** = ①②③ 켬 + Tier 1 + 안전망 ①. 2026-11-19까지의 우리 실험 결과(21~29장)는 이 설정.
- **우리 방법(지금)** = ①②③ 켬 + **Tier 2** + 안전망 ① + **안전망 ②**(2026-11-20부터, 30장).

| 무엇 | 코드 필드 | 벤치마크 옵션 | 원래 방식(코드 기본값) | 우리 방법(11-20 이전) | **우리 방법(지금)** | 설명 문서 |
|---|---|---|---|---|---|---|
| **① 고정 로봇 먼저 적기** | `order` = `kPriority` / `kFixedFirst` | `--order priority\|fixed-first` | `kPriority`(섞어 등록) | `kFixedFirst` | **`kFixedFirst`** | 12장 |
| **② 도달 확인** | `check_reachability` | `--reachability` | 끔 | 켬 | **켬** | 13장 |
| **③ 구조 로봇 고르기** | `rescue_selection` = `kBlockedCells` / `kShortestPathConflicts` | `--rescue blocked\|path` | `kBlockedCells`(막혀 본 칸 전부) | `kShortestPathConflicts` | **`kShortestPathConflicts`**(최단 길 위) | 13·15장 |
| **Tier 깊이** | `max_escalation_tiers` | `--tiers K` | 1 | 1 | **2** | 06·30장 |
| **안전망 ②** (순서 바꿔 다시) | `order_restarts` | `--order-restarts N` | 0(끔) | 0(끔) | **3** | 30장 |
| 안전망 ② 난수 씨앗 | `restart_seed` | `--restart-seed S` | 0 | 0 | 0 | 30장 |
| 양보 요청 (구현됨, 꺼 둠) | `request_yield` | `--yield` | 끔 | 끔 | 끔(ablation 한 줄용) | 19장 |
| 양보 최소 지연 | `yield_min_delay` | `--yield-min-delay N` | 1 | — | — | 19장 |
| Park식 비교 기준 (구현됨, 꺼 둠) | `affected_window` | `--affected-window W` | −1(끔) | 끔 | 끔(비교 기준 실험에서만) | 28장 |

안전망 ①(전체 재계획)은 끄는 옵션이 없다 — 세 설정 모두 항상 켜져 있다(성공률 보장).
양보 요청은 효과가 작고(목적지 밖 시간 조금 감소) 계산 시간이 약 20% 늘어서 모든 실험에서 꺼 두었다(19장). 전체 재계획에는 양보 개념이 없다.

## 2. 탐색 엔진 설정 — `AStarConfig`

| 무엇 | 코드 필드 | 벤치마크 옵션 | 기본값 | 우리 실험 | 설명 문서 |
|---|---|---|---|---|---|
| 시간 상한 | `max_timestep` | `--max-timestep N` | 256 | 256(128×128은 512) | 23장 |
| 동점 규칙 | `tie_break` = `kLargerG` / `kSmallerG` / `kRandom` | `--tie-break larger-g\|smaller-g\|random:SEED` | `kLargerG` | `kLargerG`(민감도 실험에서만 바꿈) | 18·21장 |
| 무작위 동점 씨앗 | `tie_seed` | (`random:SEED`에 포함) | 0 | — | 21장 |

## 3. 세계 모델 — 작업 시간

| 무엇 | 코드 | 벤치마크 옵션 | 기본값 | 설명 |
|---|---|---|---|---|
| 작업 시간 | `Agent::service_time` + 근무표 `TaskSchedule` | `--service LO HI` (로봇마다 U[LO,HI] 정수) | 0(끔) | 17·21장. 작업 중 로봇은 모든 방법에서 안 움직임(끄는 옵션 없음, 세계 모델의 일부) |

## 4. 벤치마크 전용 옵션 (`tools/benchmark.cpp`)

| 옵션 | 하는 일 | 문서 |
|---|---|---|
| `--large` / `--huge` | 64×64(50~200대) / 128×128(300~1000대). 없으면 32×32(5~40대) | 08·25장 |
| `--agents a,b,c` | 로봇 수 직접 지정 | 25장 |
| `--repeats N` | 지도·로봇 수마다 시나리오 수(기본 50) | |
| `--threads N` | 동시에 돌릴 수. **시간을 잴 때는 1** | 20·23장 |
| `--obstacle-mode path\|standing` | 장애물 장면: 길 중간 / 우회로에 서 있는 로봇 | 23장 |
| `--dump-failures DIR` | 안전망을 쓰거나 실패한 시나리오를 JSON으로 저장 | 30장 |

## 5. GUI 옵션 (`python/gui/app.py`)

| 옵션 | 하는 일 |
|---|---|
| `--method ours\|legacy` | 오른쪽 패널: 우리 방법(위 설정, **GUI 기본값**) / 원래 방식 |
| `--service LO HI` | 작업 시간 켜기 |
| `--map`, `--agents`, `--seed` | 지도(기본 corridor), 로봇 수(기본 30), 난수 씨앗 |

## 6. 결과에 남는 값 (`ReplanResult`)

`escalation_tier`(0~2 = 그 Tier에서 스스로 풀림, −1 = 안전망 ① 또는 ②로 풀림), `order_restart`(안전망 ②로 풀렸으면 몇 번째, 아니면 0),
`replanned_ids`, `rescued_ids`, `yielded_ids`, `deferred_ids`·`event_times`(Park식), `schedule`(새 근무표).
