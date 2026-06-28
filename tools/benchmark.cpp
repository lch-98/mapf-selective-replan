// ─────────────────────────────────────────────────────────────────
// tools/benchmark.cpp
//
// 논문용 데이터 수집 도구. 맵 2종(넓은 통로/좁은 통로) x 로봇 수
// (5/10/20) x 방법(전체 재계획 vs 선택적 재계획)을 50회씩 무작위
// 시나리오로 돌려서, 성공률과 실행 시간을 CSV로 출력한다.
//
// (로봇 40대는 이 맵 크기·완전 무작위 배치 조합에서는 PBS의 고정
// 우선순위 구조상 초기 plan() 성공률이 16x16 빈 맵에서도 0%에 가까울
// 만큼 너무 낮아서 제외했다 — 맵 문제가 아니라 "뒤 순위 로봇이 앞 로봇의
// Tail Reservation에 점점 막히는" PBS 자체의 한계로 직접 확인했다.)
//
// 한 시나리오의 흐름:
//   1. plan()으로 초기 경로를 계획한다.
//   2. 장애물 1~3개를 무작위로 고르되, 그중 적어도 하나는 실제로 어느
//      로봇의 경로와 어느 시각에 겹치도록 만든다 — 그 충돌 시각을
//      current_time으로 잡는다. (장애물이 아무도 안 지나가는 시공간에
//      생기면 "사실상 장애물이 없는 것"과 같은 의미 없는 시나리오가 되므로,
//      반드시 실제로 누군가의 길을 막는 시점을 골라야 재계획 비교가 의미를
//      가진다.)
//   3. 방법 A(전체 재계획): full_replan()을 호출 — "지금 위치 기준"으로
//      agents 전체를 처음부터 다시 계획한다(영향 안 받은 로봇도 다시 탐색).
//   4. 방법 B(선택적 재계획): replan()을 호출 — 영향받은 로봇만 Tier 0/1/
//      안전망 순서로 재계획한다.
//   5. 각 방법의 성공 여부와 걸린 시간(ms)을 기록한다.
//
// 방법 A와 방법 B는 같은 초기 경로, 같은 장애물, 같은 current_time을
// 공유한다(공정한 비교를 위해 같은 시나리오에 두 방법을 모두 적용한다).
// ─────────────────────────────────────────────────────────────────
#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include "benchmark_maps.hpp"
#include "mapf/pbs.hpp"

using namespace mapf;
using namespace mapf::bench;

namespace {

struct ScenarioResult {
    bool plan_ok{false};
    bool full_replan_ok{false};
    bool selective_replan_ok{false};
    double full_replan_ms{0.0};
    double selective_replan_ms{0.0};
    int escalation_tier{0};  // selective_replan_ok일 때만 의미 있음
    int plan_attempts{1};    // 이 trial을 채우기 위해 plan()을 몇 번 시도했는지
};

// 맵의 빈 칸(벽이 아닌 칸) 전부를 모은다.
std::vector<Cell> collect_free_cells(const Map& map) {
    std::vector<Cell> cells;
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (map.is_passable(x, y)) cells.push_back(Cell{x, y});
        }
    }
    return cells;
}

// 빈 칸 목록에서 서로 겹치지 않는 num_agents개의 (start, goal) 쌍을 무작위로
// 뽑는다. start들끼리도, goal들끼리도 중복되지 않게 한다(start==goal인
// 로봇은 허용 — 04~05장 테스트에서도 다루는 정상적인 경우다).
std::vector<Agent> make_random_agents(const std::vector<Cell>& free_cells, int num_agents,
                                       std::mt19937& rng) {
    std::vector<Cell> shuffled_starts = free_cells;
    std::vector<Cell> shuffled_goals = free_cells;
    std::shuffle(shuffled_starts.begin(), shuffled_starts.end(), rng);
    std::shuffle(shuffled_goals.begin(), shuffled_goals.end(), rng);

    std::vector<Agent> agents;
    for (int i = 0; i < num_agents; ++i) {
        agents.push_back(Agent{i, shuffled_starts[i], shuffled_goals[i]});
    }
    return agents;
}

// initial의 모든 경로에서 (칸, 시각) 쌍을 전부 모은다 — "장애물이 실제로
// 누군가의 길을 막는 시점"을 고르는 데 쓰인다. 두 종류의 칸은 제외한다:
//   - start 칸(t=0): 장애물이 당장 출발도 못 하게 막아버리면 "재계획"이
//     아니라 "처음부터 못 푼다"는 별개의 의미가 된다.
//   - goal 칸(경로의 마지막): 그 로봇은 도착 후 영원히 그 자리에 머문다는
//     Tail Reservation 전제(05장 5.5절)가 있는데, 장애물이 정확히 그
//     목적지를 그 도착 시각에 막아버리면 그 로봇은 어떤 방법으로도(전체
//     재계획이든 선택적 재계획이든) 갈 곳이 없다 — "재계획으로 우회
//     가능한 충돌"이 아니라 "목적지 자체가 봉쇄된, 누구도 못 푸는 상황"
//     이므로 의미 있는 벤치마크 시나리오가 아니다.
std::vector<SpaceTimeCell> collect_path_cells(const PBSResult& initial) {
    std::vector<SpaceTimeCell> cells;
    for (const auto& [id, path] : initial) {
        for (size_t i = 0; i < path.size(); ++i) {
            if (i == 0) continue;                  // start
            if (i == path.size() - 1) continue;    // goal
            cells.push_back(path[i]);
        }
    }
    return cells;
}

// 장애물 1~3개를 고른다. 그중 정확히 하나는 collect_path_cells에서 뽑은
// "실제로 어느 로봇이 그 시각에 지나가는 칸"이고, 나머지는 그냥 빈 칸 중
// 무작위로 고른다(장애물이 여러 개 동시에 생기는 상황을 흉내내되, 적어도
// 하나는 반드시 실제 충돌을 일으키게 보장한다).
//
// out_current_time에는 "그 칸에 도착하기 한 스텝 전" 시각을 채운다 — 장애물
// 칸의 시각(chosen.t) 그대로 쓰면 안 된다. 왜냐하면 PBS::full_replan/replan은
// "current_time 시점에 로봇이 있던 칸"에서부터 재탐색을 시작하는데, 그 칸이
// 곧 장애물 칸이라면 "로봇이 이미 장애물 안에 서 있다"는 풀 수 없는 모순이
// 된다(A* 자체는 출발 칸 점유 검사를 안 해서 경로를 찾지만, 그 경로를
// 등록하는 register_path 단계에서 시작 칸이 이미 장애물 소유라 거절된다 —
// 직접 재현해서 확인한 사실이다). 우리가 원하는 의미는 "장애물이 생긴 그
// 순간, 로봇은 아직 그 칸에 도착하기 전이고 원래는 다음 스텝에 그 칸으로
// 들어갈 예정이었다"이므로, current_time은 chosen.t-1이어야 한다.
std::vector<Cell> make_obstacles_that_actually_block(const std::vector<Cell>& free_cells,
                                                       const PBSResult& initial,
                                                       std::mt19937& rng, int* out_current_time) {
    std::vector<SpaceTimeCell> path_cells = collect_path_cells(initial);
    // path_cells가 비는 경우(모든 로봇이 1~2스텝짜리 짧은 경로일 때 흔함)는
    // 충돌시킬 시공간이 없으므로, 장애물을 빈 칸 중 무작위로 두고
    // current_time=0으로 둔다 — 이 경우 1단계(영향받는 로봇 판별)에서
    // 아무도 영향을 안 받는 것으로 정확히 처리된다(의미상 모순이 아니다).
    Cell blocking_cell = free_cells.front();
    int current_time = 0;
    if (!path_cells.empty()) {
        std::uniform_int_distribution<size_t> idx_dist(0, path_cells.size() - 1);
        const SpaceTimeCell& chosen = path_cells[idx_dist(rng)];
        blocking_cell = Cell{chosen.x, chosen.y};
        current_time = chosen.t - 1;
    }
    *out_current_time = current_time;

    std::vector<Cell> obstacles = {blocking_cell};

    // 나머지 0~2개는 그냥 무작위 빈 칸(중복/시작칸 제외)으로 채운다.
    std::vector<Cell> candidates;
    for (const Cell& cell : free_cells) {
        if (cell == blocking_cell) continue;
        candidates.push_back(cell);
    }
    std::shuffle(candidates.begin(), candidates.end(), rng);

    std::uniform_int_distribution<int> extra_count_dist(0, 2);
    int extra_count = extra_count_dist(rng);
    for (int i = 0; i < extra_count && i < static_cast<int>(candidates.size()); ++i) {
        obstacles.push_back(candidates[static_cast<size_t>(i)]);
    }
    return obstacles;
}

double elapsed_ms(std::chrono::steady_clock::time_point start) {
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// 전방 선언 — find_solvable_scenario_and_run이 아래쪽에 정의된
// run_one_scenario를 먼저 호출하기 때문에 필요하다.
ScenarioResult run_one_scenario(const Map& map, const std::vector<Agent>& agents,
                                 const PBSResult& initial, std::mt19937& rng);

// plan()에 성공하는 시나리오를 찾을 때까지 무작위로 agents를 다시 뽑는다.
// (A안) "장애물이 생기기도 전에 초기 계획 자체가 실패하는" 시나리오는
// 재계획 비교(방법 A vs B)와는 별개의 문제이므로 표본에서 제외한다 — 그래야
// 모든 trial이 "일단 다들 갈 길이 있던 정상적인 상황"에서 출발해, 방법 A와
// 방법 B를 공정하게 비교할 수 있다. 대신 몇 번 재시도해야 했는지
// (plan_attempts)를 따로 기록해서, "로봇 수가 늘수록 초기 설계 자체가
// 어려워진다"는 사실 자체를 잃어버리지 않고 CSV에 명시한다.
//
// max_attempts번 안에 못 찾으면(극히 드문 경우) 마지막 실패를 그대로
// plan_ok=false로 기록하고 멈춘다 — 무한 루프를 막기 위한 안전장치다.
ScenarioResult find_solvable_scenario_and_run(const Map& map, int num_agents, std::mt19937& rng) {
    constexpr int kMaxAttempts = 200;

    std::vector<Cell> free_cells = collect_free_cells(map);

    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        std::vector<Agent> agents = make_random_agents(free_cells, num_agents, rng);

        PBS pbs(map);
        std::optional<PBSResult> initial = pbs.plan(agents);
        if (!initial.has_value()) {
            if (attempt == kMaxAttempts) {
                ScenarioResult failed;
                failed.plan_ok = false;
                failed.plan_attempts = attempt;
                return failed;
            }
            continue;  // 이번 무작위 배치는 초기 계획부터 실패 — 다시 뽑는다.
        }

        ScenarioResult result = run_one_scenario(map, agents, *initial, rng);
        result.plan_attempts = attempt;
        return result;
    }

    ScenarioResult failed;
    failed.plan_ok = false;
    failed.plan_attempts = kMaxAttempts;
    return failed;
}

// 한 무작위 시나리오를 만들고, 방법 A/B를 모두 돌려서 결과를 기록한다.
// 호출하는 쪽(find_solvable_scenario_and_run)이 이미 "plan()에 성공하는
// 시나리오"를 찾아서 agents/initial을 넘겨주므로, 여기서는 plan_ok=true가
// 항상 보장된다.
ScenarioResult run_one_scenario(const Map& map, const std::vector<Agent>& agents,
                                 const PBSResult& initial, std::mt19937& rng) {
    ScenarioResult result;
    result.plan_ok = true;

    std::vector<Cell> free_cells = collect_free_cells(map);
    int current_time = 0;
    std::vector<Cell> obstacles =
        make_obstacles_that_actually_block(free_cells, initial, rng, &current_time);

    // 방법 A: 전체 재계획 (현재 위치 기준).
    {
        PBS pbs_a(map);
        auto t0 = std::chrono::steady_clock::now();
        std::optional<PBSResult> full = pbs_a.full_replan(agents, initial, obstacles, current_time);
        result.full_replan_ms = elapsed_ms(t0);
        result.full_replan_ok = full.has_value();
    }

    // 방법 B: 선택적 재계획(Tier 0/1/안전망).
    {
        PBS pbs_b(map);
        auto t0 = std::chrono::steady_clock::now();
        std::optional<ReplanResult> selective = pbs_b.replan(agents, initial, obstacles, current_time);
        result.selective_replan_ms = elapsed_ms(t0);
        result.selective_replan_ok = selective.has_value();
        if (selective.has_value()) result.escalation_tier = selective->escalation_tier;
    }

    return result;
}

}  // namespace

int main() {
    const std::vector<int> agent_counts = {5, 10, 20};
    const int kRepeats = 50;

    struct NamedMap {
        const char* name;
        Map map;
    };
    std::vector<NamedMap> maps;
    maps.push_back({"open", make_open_map()});
    maps.push_back({"corridor", make_corridor_map()});

    // (A안 명시) 이 CSV의 모든 행은 "장애물이 생기기 전, 초기 plan()에
    // 이미 성공한 시나리오"만 표본으로 삼는다 — 초기 계획 자체가 실패하는
    // 무작위 배치는 재시도해서 버리고 다시 뽑는다(find_solvable_scenario_and_run).
    // plan_attempts 컬럼이 "이 trial을 채우려고 몇 번 다시 뽑았는지"를
    // 보여준다 — 1보다 크면 그만큼 초기 설계가 어려웠다는 뜻이고, 이 값이
    // 로봇 수가 늘수록 커지는 추세 자체가 "초기 설계 난이도" 지표로 쓸 수 있다.
    std::cerr << "[note] every row's initial plan() already succeeded — scenarios where the "
                 "initial plan failed were retried (see plan_attempts column).\n";

    std::cout << "map,num_agents,trial,plan_attempts,plan_ok,full_replan_ok,full_replan_ms,"
                 "selective_replan_ok,selective_replan_ms,escalation_tier\n";

    std::mt19937 rng(12345);  // 고정 시드 — 실행마다 같은 시나리오 재현 가능.

    for (const NamedMap& named_map : maps) {
        for (int num_agents : agent_counts) {
            for (int trial = 0; trial < kRepeats; ++trial) {
                ScenarioResult r = find_solvable_scenario_and_run(named_map.map, num_agents, rng);

                std::cout << named_map.name << "," << num_agents << "," << trial << ","
                          << r.plan_attempts << "," << (r.plan_ok ? 1 : 0) << ","
                          << (r.full_replan_ok ? 1 : 0) << "," << r.full_replan_ms << ","
                          << (r.selective_replan_ok ? 1 : 0) << "," << r.selective_replan_ms << ","
                          << r.escalation_tier << "\n";
            }
        }
    }

    return 0;
}
