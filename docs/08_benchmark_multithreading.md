# 08. 벤치마크 도구의 멀티스레드화 — 기초부터 버그까지 전부

이 문서는 `tools/benchmark.cpp`에 멀티스레드 코드를 추가한 세션(커밋 `cc361b3`,
직전 `4ebc8d3`)을 처음부터 끝까지 복기한다. C++ 멀티스레딩을 전혀 모른다고
가정하고, ① 기초 개념 → ② 이 파일에 정확히 무엇을 추가했는지(라인 단위) →
③ 그 과정에서 발견하고 고친 버그들 순서로 진행한다.

---

## 0. 그날 있었던 일의 큰 흐름

1. 벤치마크(맵 2종 × 로봇 5/10/20 × 50회 = 300번 시나리오)를 순차 실행했더니
   90~100초가 걸렸다.
2. 사용자가 "멀티스레드로 속도를 올리면 안 되냐"고 요청.
3. `std::thread` + 작업 큐로 병렬화했는데, **CPU는 1058%까지 쓰는데 실행
   시간은 거의 그대로**인 이상한 현상이 발생.
4. 원인을 추적하다가 "한 trial이 43.7초 걸려서 혼자 전체 실행 시간을
   지배하고 있다"는 사실을 발견.
5. 그 trial을 더 파고드니, 벤치마크의 장애물 생성 로직 자체에 **로직
   버그**가 있었다(다른 로봇의 목적지를 장애물로 막아버리는 경우를
   빠뜨림 — A*가 "절대 못 찾는 길"을 max_timestep까지 다 뒤지느라 수십 초를
   허비).
6. 그 버그를 고치고 나니 90~100초 → 24초로 줄었다(대부분 버그 수정 덕분,
   멀티스레드는 보너스).

즉 이번 세션의 진짜 교훈은: **"병렬화해도 안 빨라진다"는 신호가 오히려
숨어있던 순차 로직 버그(하나의 극단적으로 느린 작업)를 찾아내는 단서가
됐다**는 것이다.

---

## 1. C++ 멀티스레딩 기초 개념

### 1.1 스레드(thread)란?

프로그램이 실행될 때 CPU가 명령어를 하나씩 순서대로 처리하는 흐름을
"스레드"라고 부른다. 보통 `main()`은 스레드 1개로 시작한다(이걸 "메인
스레드"라 부른다). `std::thread`를 쓰면 **같은 프로그램 안에서 동시에 여러
개의 실행 흐름을 만들 수 있다** — 예를 들어 스레드 8개를 만들면, CPU 코어가
8개 이상일 때 실제로 8개의 계산이 동시에 진행된다.

비유: 요리사 1명이 재료 300개를 순서대로 손질하면 오래 걸리지만, 요리사
8명이 나눠서 손질하면(같은 부엌, 같은 도마 개수만 조심하면) 훨씬 빨리
끝난다. 여기서 "요리사"가 스레드, "재료 300개"가 벤치마크의 300개 trial이다.

### 1.2 왜 멀티스레드가 필요했나 — 이 프로젝트의 상황

이 벤치마크는 **서로 완전히 독립적인 300번의 시뮬레이션**을 돌린다(맵
2종 × 로봇수 3종 × 50회 반복). trial 하나의 결과가 다른 trial에 전혀
영향을 주지 않는다 — 이런 작업을 "**embarrassingly parallel**(병렬화가
쉬운 작업)"이라고 부른다. 서로 대화할 필요가 없는 작업들이므로, 이론적으로
스레드 개수만큼 거의 그대로 속도가 올라가야 정상이다.

### 1.3 핵심 개념 4가지 (이번 코드에서 실제로 쓰인 것만)

**(1) `std::thread` — 스레드를 만들고 합류(join)시키기**

```cpp
std::thread t(어떤_함수);  // 새 실행 흐름 시작
t.join();                  // 이 스레드가 끝날 때까지 기다림
```
`join()`을 안 하면 메인 스레드가 먼저 끝나버려서 프로그램이 강제 종료될
수 있다 — 그래서 항상 만든 스레드는 join으로 "수거"해야 한다.

**(2) 경쟁 상태(race condition)와 `std::atomic`**

두 스레드가 **동시에 같은 변수를 읽고 쓰면** 결과가 꼬일 수 있다. 예를 들어
`int counter`를 두 스레드가 동시에 `counter++`하면, 운이 나쁘면 한 번의
증가가 유실될 수 있다(각 스레드가 "읽기 → 더하기 → 쓰기"를 하는데 그 사이에
끼어들 수 있어서). 이걸 막는 가장 간단한 도구가 `std::atomic<size_t>`다 —
"읽고-더하고-쓰는" 것을 **하나의 끊기지 않는 동작**으로 만들어준다.

**(3) 작업 큐(work queue) 패턴 — "누가 몇 번째 일을 할지"**

스레드 개수(예: 8개)보다 작업 개수(300개)가 훨씬 많을 때, 각 스레드가
"다음으로 비어있는 작업 번호"를 하나씩 뽑아가는 방식을 쓴다. `std::atomic`
카운터 하나로 이 "번호표 뽑기"를 구현할 수 있다 — 여러 스레드가 동시에
`fetch_add(1)`을 호출해도 절대 같은 번호를 두 스레드가 받는 일이 없다.

**(4) 표준출력(`std::cout`)은 스레드 안전하지 않다**

여러 스레드가 동시에 `std::cout << "..."`을 호출하면, 한 줄 안에 다른
스레드의 글자가 섞여 들어갈 수 있다(예: `map=open,agemap=corridor,...`처럼
깨짐). 그래서 "각 스레드는 결과를 자기만의 문자열에 모아두고, 마지막에
메인 스레드 혼자서 순서대로 출력"하는 패턴을 쓴다.

### 1.4 이번 설계에서 특히 중요한 개념 — "결정론적 병렬화"

멀티스레드 코드는 보통 "실행할 때마다 결과가 조금씩 달라질 수 있다"는
인상이 있다(스레드 순서가 매번 다르므로). 하지만 이 벤치마크는 **몇 번을
실행해도 완전히 똑같은 CSV가 나와야** 연구 데이터로 신뢰할 수 있다.
방법은 간단하다 — **"몇 번째 스레드가 이 작업을 처리했는가"가 결과에
전혀 영향을 주지 않도록 만들면 된다.** 구체적으로는, 각 작업(task)마다
난수 시드(random seed)를 "그 작업의 고유 번호"로 미리 정해두고, 그
시드로 그 작업 전용의 독립적인 난수 생성기(`std::mt19937`)를 새로 만든다.
그러면 어느 스레드가 언제 처리하든 항상 같은 무작위 시나리오가 나온다.

---

## 2. `tools/benchmark.cpp`에 정확히 무엇을 추가했는가 (라인 단위)

아래는 이전 커밋(`4ebc8d3`, 순차 버전)과 이번 커밋(`cc361b3`, 멀티스레드
버전)의 diff를 실제로 대조해서 확인한 내용이다.

### 2.1 헤더 추가 ([benchmark.cpp:37-43](../tools/benchmark.cpp#L37-L43))

```cpp
#include <atomic>    // std::atomic — 여러 스레드가 안전하게 공유하는 카운터
#include <sstream>   // std::ostringstream — 결과를 문자열로 모으는 용도
#include <thread>    // std::thread — 스레드 생성/join
```

### 2.2 `Task` 구조체 — "작업 하나"를 표현 ([benchmark.cpp:312-318](../tools/benchmark.cpp#L312-L318))

```cpp
struct Task {
    const Map* map;        // 어느 맵인지 (포인터 — 복사 비용 없이 참조만)
    const char* map_name;  // CSV에 찍을 이름("open"/"corridor")
    int num_agents;        // 로봇 몇 대
    int trial;             // 몇 번째 반복인지 (0~49)
    uint32_t seed;         // 이 task 전용 난수 시드
};
```

`map`을 값이 아니라 **포인터**로 들고 있는 이유: `Map`은 32×32 격자
데이터를 담은 꽤 큰 객체다. 300개의 Task를 만들 때마다 이걸 통째로
복사하면 낭비이므로, `main()`의 `maps` 벡터 안에 원본을 계속 살려두고
포인터로만 가리킨다.

### 2.3 `run_task` — 작업 하나를 처리해서 CSV 한 줄을 "문자열로" 반환 ([benchmark.cpp:325-335](../tools/benchmark.cpp#L325-L335))

```cpp
std::string run_task(const Task& task) {
    std::mt19937 rng(task.seed);  // 이 task만의 독립 RNG
    ScenarioResult r = find_solvable_scenario_and_run(*task.map, task.num_agents, rng);

    std::ostringstream out;
    out << task.map_name << "," << task.num_agents << "," << task.trial << ","
        << r.plan_attempts << "," << (r.plan_ok ? 1 : 0) << "," << (r.full_replan_ok ? 1 : 0)
        << "," << r.full_replan_ms << "," << (r.selective_replan_ok ? 1 : 0) << ","
        << r.selective_replan_ms << "," << r.escalation_tier << "\n";
    return out.str();
}
```

이전 버전(순차)에서는 `main()` 안의 3중 for문에서 직접 시나리오를 계산하고
바로 `std::cout`에 찍었다. 이번엔 그 로직을 `run_task`라는 별도 함수로
뽑아내서, **`std::cout`에 안 쓰고 문자열로 반환**하도록 바꿨다. 이게 위
1.3절 (4)번 개념 그대로다.

### 2.4 `main()` 내부 — 작업 목록 만들기 ([benchmark.cpp:360-374](../tools/benchmark.cpp#L360-L374))

```cpp
constexpr uint32_t kBaseSeed = 12345;
std::vector<Task> tasks;
for (const NamedMap& named_map : maps) {
    for (int num_agents : agent_counts) {
        for (int trial = 0; trial < kRepeats; ++trial) {
            uint32_t seed = kBaseSeed + static_cast<uint32_t>(tasks.size());
            tasks.push_back(Task{&named_map.map, named_map.name, num_agents, trial, seed});
        }
    }
}
```

이전 버전은 이 3중 for문 **안에서 바로 계산**했다. 이번엔 "계산은 나중에,
지금은 할 일 목록만 만든다"로 바뀌었다. `seed = kBaseSeed + tasks.size()`가
핵심 트릭이다 — `tasks.size()`는 지금까지 만든 task 개수이므로, 이게 곧
"이 task가 목록에서 몇 번째인가"가 되고, 그 번호를 그대로 시드에 더한다.
그러면 **목록에서의 위치만으로 시드가 정해지고, 실행 순서와는 무관**해진다
→ 이게 위 1.4절 "결정론적 병렬화"의 실제 구현이다.

### 2.5 결과를 담을 자리를 미리 만들기 ([benchmark.cpp:379-380](../tools/benchmark.cpp#L379-L380))

```cpp
std::vector<std::string> results(tasks.size());  // task 개수만큼 빈 문자열 자리 미리 확보
std::atomic<size_t> next_task_index{0};           // "다음으로 처리할 task 번호" 공유 카운터
```

`std::vector<std::string> results(tasks.size())`는 크기 300짜리 벡터를
**미리 그 자리에 만들어둔다**(빈 문자열 300개로 초기화). 이렇게 하면 나중에
`results[i] = ...`로 채울 때 "몇 번째 스레드가 몇 번째 task를 처리했는지"와
무관하게 항상 `results[i]`가 정확히 task `i`의 결과가 된다.

`next_task_index`가 바로 위 1.3절 (3)번 "작업 큐"의 구현체다.

### 2.6 워커(worker) 람다 — 실제로 스레드가 실행할 코드 ([benchmark.cpp:389-395](../tools/benchmark.cpp#L389-L395))

```cpp
auto worker = [&]() {
    while (true) {
        size_t i = next_task_index.fetch_add(1);  // "내 번호표"를 뽑고 카운터를 1 증가
        if (i >= tasks.size()) break;              // 더 이상 할 일이 없으면 종료
        results[i] = run_task(tasks[i]);
    }
};
```

`fetch_add(1)`이 정확히 무슨 일을 하는지 짚어보자: **"현재 값을 반환하면서
동시에 1을 더한다"**를 **끊기지 않게(atomic)** 수행한다. 그래서 스레드
A와 B가 동시에 이 줄을 실행해도, 한쪽은 반드시 0을 받고 다른 쪽은 반드시
1을 받는다 — 절대 둘 다 0을 받는 일이 없다. 이게 없으면(예를 들어 그냥
`int`를 썼다면) 두 스레드가 동시에 같은 task를 두 번 처리하거나, 어떤
task는 아무도 처리하지 않는 사고가 날 수 있다.

`[&]`는 "바깥 변수들(`results`, `tasks`, `next_task_index`)을 참조로
캡처한다"는 뜻이다 — 람다 함수가 자기 바깥 스코프의 변수를 그대로 가리켜
쓸 수 있게 해준다.

### 2.7 스레드 개수 정하기 ([benchmark.cpp:382-387](../tools/benchmark.cpp#L382-L387))

```cpp
unsigned int num_threads = std::thread::hardware_concurrency();
if (num_threads == 0) num_threads = 4;  // 알 수 없으면 4로 폴백
num_threads = std::min<unsigned int>(num_threads, static_cast<unsigned int>(tasks.size()));
```

`hardware_concurrency()`는 "이 컴퓨터가 동시에 처리할 수 있는 하드웨어
스레드 개수"(대략 CPU 코어 수, 하이퍼스레딩 포함)를 알려준다. 드물게
이 정보를 못 가져오면 0을 반환하는데, 그럴 때를 대비해 4로 기본값을
둔다. 마지막 줄은 "작업이 3개뿐인데 스레드를 16개 만드는" 낭비를 막는다
(스레드를 만드는 것 자체도 공짜가 아니다).

### 2.8 스레드 생성과 join ([benchmark.cpp:397-399](../tools/benchmark.cpp#L397-L399))

```cpp
std::vector<std::thread> workers;
for (unsigned int t = 0; t < num_threads; ++t) workers.emplace_back(worker);
for (std::thread& t : workers) t.join();
```

`num_threads`개의 스레드를 만들어서 전부 같은 `worker` 람다를 실행시킨다.
각 스레드는 독립적으로 `next_task_index`에서 번호표를 뽑아가며 일하다가,
더 이상 할 일이 없으면(`i >= tasks.size()`) 스스로 종료한다. 그다음
`join()` 루프에서 "모든 스레드가 완전히 끝날 때까지" 메인 스레드가
기다린다 — 이걸 안 하면 아직 `results`를 채우고 있는 스레드가 있는데
메인 스레드가 먼저 출력을 시작해버릴 수 있다.

### 2.9 마지막 출력 — 순서 보장 ([benchmark.cpp:401-403](../tools/benchmark.cpp#L401-L403))

```cpp
std::cout << "map,num_agents,trial,...\n";
for (const std::string& line : results) std::cout << line;
```

`results`는 처음부터 task 순서대로 자리가 잡혀 있었으므로(2.5절), 이
루프는 항상 "맵 → 로봇수 → trial" 순서로 출력된다 — 스레드가 실제로
어떤 순서로 일을 끝냈는지와 완전히 무관하다.

### 2.10 정리 — 왜 이 설계가 "말이 되는지" 한 문장으로

**"할 일 목록을 미리 확정하고(시드까지), 각 스레드는 번호표를 뽑아 자기
결과를 자기 칸에만 쓰고, 다 끝나면 칸 순서대로 출력한다"** — 이 세 가지
규칙만 지키면 스레드가 몇 개든, 어떤 순서로 끝나든 항상 같은 CSV가
나온다.

### 2.11 CMake 변경 — pthread 링크

멀티스레드 코드는 `tools/CMakeLists.txt`에 다음이 추가되어야 링크 에러 없이
빌드된다(Windows/MSVC는 스레드가 기본 런타임에 포함되어 있어 문제가 잘
안 드러나지만, 나중에 Ubuntu/g++로 옮길 때 `-lpthread`가 없으면 링크
에러가 난다):

```cmake
find_package(Threads REQUIRED)
target_link_libraries(run_benchmark PRIVATE mapf_core Threads::Threads)
```

---

## 3. 발견하고 고친 버그들 — 처음부터 끝까지

이번 세션에서는 **두 가지 서로 다른 문제**가 얽혀 있었다. 순서대로
설명한다.

### 버그 A: "장애물이 실제로 기존 경로를 막는지 검증 안 함" (멀티스레드와는 별개, 먼저 지적된 문제)

**사용자의 원래 질문 취지**: "경로계획 성공 → 장애물 등장 → 그 장애물
때문에 실제로 실패하는 시나리오에 대해서만 full_replan과 replan을
비교해야 하는 거 아니냐?"

**코드를 다시 보니 실제로 그랬다**: 이전 버전(`4ebc8d3`)의
`make_obstacles_that_actually_block`은 "기존 경로의 중간 칸"에서 장애물을
골랐지만, 그게 **정말로** 누군가의 경로를 막는지 직접 검증하는 코드는
없었다 — "중간 칸이니까 당연히 막겠지"라고 추론만 하고 있었다.

**고친 방법**:
1. `PBS::path_hits_obstacle`을 `private`에서 `public static`으로 승격
   (`core/include/mapf/pbs.hpp`) — 이 함수는 `replan()`이 내부적으로
   "이 로봇이 이 장애물 때문에 영향받는가"를 판단할 때 쓰는 바로 그
   함수다. 벤치마크가 **똑같은 함수**를 그대로 재사용하게 만들어서,
   "벤치마크가 판단하는 기준"과 "라이브러리가 실제로 판단하는 기준"이
   어긋날 위험을 원천 차단했다.
2. [benchmark.cpp:207-213](../tools/benchmark.cpp#L207-L213)에
   `obstacles_actually_block_someone` 함수를 추가 — 모든 로봇에 대해
   `PBS::path_hits_obstacle`을 호출해서 한 명이라도 막히면 `true`.
3. [benchmark.cpp:284-296](../tools/benchmark.cpp#L284-L296)에서, 장애물을
   고른 직후 이 함수로 검증하고, 아무도 안 막히면 그 시나리오를 버리고
   재시도한다.

이때 사용자가 동시에 "full_replan이 실패할 때만 replan을 실행하는 거
아니냐"고도 물었는데, 이건 오해였다 — 실제 설계는 **항상 둘 다 독립
실행**해서 같은 시나리오를 비교하는 것이 맞다(그래야 공정한 비교가 된다).
이 두 질문은 서로 다른 내용이라 혼동하지 않도록 구분해서 기록해둔다.

### 버그 B: 멀티스레드화 후에도 안 빨라짐 → 43.7초짜리 trial 발견

**증상**: `std::thread`로 병렬화했는데 CPU 사용률은 1058%(코어 10개 넘게
활용 중)인데 벽시계 시간(wall time)은 거의 줄지 않음.

**추적 과정**:
1. Windows에서 `Get-Counter`로 CPU 사용률을 직접 확인 → 확실히 여러
   코어를 쓰고는 있음. 그렇다면 "작업 분배가 불균등하다"는 가설이 남는다
   — 즉, 어떤 작업 하나가 극단적으로 오래 걸려서 그 작업을 맡은 스레드
   혼자 다른 스레드들이 다 끝난 뒤에도 한참을 더 도는 상황.
2. CSV 결과에서 `selective_replan_ms` 컬럼을 내림차순 정렬 → 최댓값이
   **43731ms(43.7초)**. 다른 값들은 대부분 수십 ms 수준이었으므로, 이
   trial 하나가 전체 실행 시간을 사실상 결정하고 있었다.
3. 이 현상을 재현하기 위해 `tools/quick_check.cpp`라는 임시 디버깅 파일을
   만들어서(에이전트 메모리에 따르면 이 패턴 — 임시 파일 만들고 디버깅
   후 삭제 — 을 이 프로젝트에서 반복 사용함), 문제의 agent 하나만 따로
   떼서 A* 탐색 시간을 측정해봤다.
4. 결과: 특정 agent 하나가 6.8초 동안 실패로 끝남. 즉 "A*가 6.8초 동안
   존재하지도 않는 경로를 찾아 헤매다가 결국 실패로 판정"되고 있었다.

**근본 원인**: [benchmark.cpp:148-195](../tools/benchmark.cpp#L148-L195)의
`make_obstacles_that_actually_block`이 장애물 후보를 고를 때, "그 칸을
장애물로 삼을 로봇 **자기 자신**의 goal"만 피하고 있었다(옛 코드 기준).
그런데 **다른 로봇의 목적지**와 우연히 같은 칸을 장애물로 골라버리면,
그 다른 로봇은 영원히 도착할 수 없는 상태가 된다. A* 알고리즘은 "이
목적지는 원래 도달 불가능하다"는 사실을 미리 알 방법이 없으므로,
`max_timestep`(256)까지 가능한 모든 경로를 전부 탐색해보고 나서야 "실패"라고
결론 낸다 — 이게 바로 수 초~수십 초가 걸리는 이유다.

**고친 방법**: `collect_path_cells`와 `make_obstacles_that_actually_block`
둘 다 `agents` 전체(따라서 모든 로봇의 goal 집합)를 받도록 시그니처를
바꿨다.
- [benchmark.cpp:107-127](../tools/benchmark.cpp#L107-L127)
  `collect_path_cells(initial, all_goals)` — 장애물 후보 칸이 **어느
  로봇의 목적지와도** 겹치면 후보에서 제외([benchmark.cpp:115-122](../tools/benchmark.cpp#L115-L122)).
- [benchmark.cpp:148-195](../tools/benchmark.cpp#L148-L195)
  `make_obstacles_that_actually_block` — 추가 장애물(0~2개)을 고를 때도
  같은 검사를 적용([benchmark.cpp:177-184](../tools/benchmark.cpp#L177-L184)).

**결과 검증**: 버그 수정 후 90~100초 → 24초로 줄었다(멀티스레드 자체의
효과보다 이 버그 수정의 효과가 훨씬 컸다 — hung trial 하나가 사실상 전체를
직렬화시키고 있었으므로). 재현성도 직접 확인했다 — 같은 `kBaseSeed`로
두 번 실행해서 시간(ms) 컬럼만 빼고 나머지(`plan_ok`, `full_replan_ok`,
`selective_replan_ok`, `escalation_tier`, `plan_attempts`)를 diff하면
완전히 동일했다.

### 왜 이 두 버그가 "멀티스레드 때문에 생긴 버그"가 아닌지

중요한 점: 버그 A와 버그 B는 **멀티스레드 코드 자체의 버그가 아니다.**
둘 다 순차 버전에도 이미 있던 로직 버그였고, 멀티스레드화는 그저 버그
B를 "체감상 눈에 띄게" 만든 계기였을 뿐이다(순차 버전이었어도 43.7초짜리
trial은 똑같이 있었겠지만, "전체 100초 중 43초"라 티가 덜 났을 것이다).
실제로 이 세션에서 작성한 멀티스레드 코드(`std::thread`, `std::atomic`,
작업 큐 부분) 자체에서는 새로운 버그가 발견되지 않았다 — race condition,
deadlock 같은 전형적인 동시성 버그는 없었다.

---

## 4. 전체를 한 문장씩으로 요약

1. 멀티스레드는 "서로 독립적인 300개의 작업을 여러 스레드가 나눠 갖는"
   작업 큐 패턴으로 구현했고, 시드를 작업 순번으로 고정해 실행 순서와
   무관하게 항상 같은 결과가 나오게 만들었다.
2. 결과는 `std::cout`에 바로 안 쓰고 각자 문자열로 모았다가 마지막에
   순서대로 출력해서, 여러 스레드가 동시에 출력해 줄이 섞이는 문제를
   피했다.
3. 병렬화를 했는데도 안 빨라진 이유는 동시성 버그가 아니라, 벤치마크의
   장애물 생성 로직이 "다른 로봇의 목적지를 실수로 막아버려서 A*가
   수십 초간 헛수고하는" 순차 로직 버그 때문이었다.
4. 그 김에 "장애물이 실제로 누군가의 경로를 막는지"를 라이브러리의
   실제 판별 함수(`PBS::path_hits_obstacle`)로 직접 검증하도록 만들어서,
   벤치마크가 측정하는 시나리오의 정당성 자체도 함께 강화했다.

---

## 스스로 확인하는 질문

1. `std::atomic<size_t> next_task_index`를 그냥 `size_t`로 바꾸면 왜
   위험한가? (힌트: 두 스레드가 동시에 같은 숫자를 읽어갈 수 있다는
   시나리오를 떠올려보자.)
2. `results[i] = run_task(tasks[i]);`를 `std::cout << run_task(tasks[i]);`로
   바꾸면 어떤 문제가 생기는가?
3. `seed = kBaseSeed + tasks.size()` 방식 대신 "각 스레드가 시작할 때
   `std::random_device`로 매번 새로 시드를 뽑는" 방식을 썼다면, 이
   벤치마크의 어떤 목표(2.10절 재현성)가 깨지는가?
4. 버그 B(6.8초 걸리는 A* 실패)가 만약 `max_timestep`이 256이 아니라
   10000이었다면 어떻게 됐을까?
