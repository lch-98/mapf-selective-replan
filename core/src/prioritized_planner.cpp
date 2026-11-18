// ─────────────────────────────────────────────────────────────────
// core/src/prioritized_planner.cpp
//
// prioritized_planner.hpp에서 선언한 PrioritizedPlanner::plan / PrioritizedPlanner::replan의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/prioritized_planner.hpp"

#include <algorithm>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace mapf {

namespace {

// 재계획한 경로를 만든다: 옛 경로의 과거 구간(t < current_time) + 새로 찾은
// 미래 구간(t >= current_time). 로봇이 이미 도착해서 옛 경로가 current_time
// 전에 끝났다면(Tail 상태), 그 사이 시각에는 목적지에 머물러 있었으므로 그
// 칸으로 채워 넣는다 — 빠뜨리면 시각이 건너뛰어 "경로의 i번째 칸 = 시각 i"
// 라는 전제(position_at, GUI의 sim_clock 등)가 깨지고 로봇 위치를 잘못 읽는다.
Path splice_paths(const Path& old_path, const Path& new_future, int current_time) {
    Path full_path;
    for (const SpaceTimeCell& cell : old_path) {
        if (cell.t >= current_time) break;
        full_path.push_back(cell);
    }
    if (!full_path.empty()) {
        const SpaceTimeCell last = full_path.back();
        for (int t = last.t + 1; t < current_time; ++t) {
            full_path.push_back(SpaceTimeCell{last.x, last.y, t});
        }
    }
    for (const SpaceTimeCell& cell : new_future) {
        full_path.push_back(cell);
    }
    return full_path;
}

// 다른 로봇을 전혀 무시하고, 지도의 벽과 blocked 칸(새 장애물)만 피해서 from → to 최단
// 경로를 BFS로 찾는다(13장). 반환값은 from부터 to까지의 칸 목록이고, 못 가면 빈 목록이다.
// 시간 개념이 없는 평범한 BFS라 대기는 필요 없다. 이웃을 늘 같은 순서로 보므로 같은
// 입력에는 항상 같은 경로가 나온다.
std::vector<Cell> static_shortest_path(const Map& map, Cell from, Cell to,
                                       const std::vector<Cell>& blocked) {
    auto is_blocked = [&blocked](Cell c) {
        return std::find(blocked.begin(), blocked.end(), c) != blocked.end();
    };
    if (is_blocked(to)) return {};

    const int width = map.width();
    auto index = [width](Cell c) { return static_cast<size_t>(c.y * width + c.x); };
    constexpr int kUnvisited = -2;
    constexpr int kStart = -1;
    std::vector<int> parent(static_cast<size_t>(width * map.height()), kUnvisited);

    std::queue<Cell> frontier;
    parent[index(from)] = kStart;
    frontier.push(from);
    while (!frontier.empty()) {
        Cell current = frontier.front();
        frontier.pop();
        if (current == to) break;
        for (const Cell& next : map.neighbors(current)) {
            if (next == current) continue;  // 대기 동작은 BFS에 필요 없다
            if (parent[index(next)] != kUnvisited || is_blocked(next)) continue;
            parent[index(next)] = static_cast<int>(index(current));
            frontier.push(next);
        }
    }
    if (parent[index(to)] == kUnvisited) return {};

    std::vector<Cell> route;
    for (int i = static_cast<int>(index(to)); i != kStart; i = parent[static_cast<size_t>(i)]) {
        route.push_back(Cell{i % width, i / width});
    }
    std::reverse(route.begin(), route.end());
    return route;
}

// 재계획에서 로봇 한 대를 "몇 시부터, 작업을 얼마나 남기고" 다시 계획하는지(17장 결정 6).
struct ReplanStart {
    int time;     // 이 시각의 위치에서 A*를 시작한다. 이 시각 전의 옛 경로는 그대로 둔다
    int service;  // A*에 넘기는 작업 시간(도착하면 이만큼 목적지에 머문다)
};

ReplanStart replan_start(const Agent& agent, const TaskWindow& window, int current_time) {
    switch (task_state_at(window, current_time)) {
        case TaskState::kMoving:   // 아직 도착 전: 지금부터, 도착하면 작업해야 한다
            return ReplanStart{current_time, agent.service_time};
        case TaskState::kWorking:  // 작업 중: 작업이 끝나는 시각부터. 작업은 이미 하고 있다
            return ReplanStart{window.end, 0};
        case TaskState::kParked:   // 주차 중: 지금부터. 작업은 이미 끝났다
            break;
    }
    return ReplanStart{current_time, 0};
}

// 근무표 없이 부르는 예전 replan/full_replan용. 작업 시간이 하나라도 있으면 거절한다.
void require_zero_service(const std::vector<Agent>& agents, const char* function_name) {
    for (const Agent& agent : agents) {
        if (agent.service_time != 0) {
            throw std::invalid_argument(std::string("PrioritizedPlanner::") + function_name +
                                        ": agents with service_time need a TaskSchedule "
                                        "(use the overload that takes previous_schedule)");
        }
    }
}

// 작업 시간이 전부 0일 때의 근무표: 도착 = 작업 끝 = 경로 끝. 작업 중인 순간이 없으므로
// window_from_path처럼 경로 모양을 검사할 필요가 없다(실패 후 멈춰 선 경로처럼 목적지에서
// 끝나지 않는 경로가 와도 예전과 똑같이 동작한다).
TaskSchedule zero_service_schedule(const std::vector<Agent>& agents, const PlanResult& paths) {
    TaskSchedule schedule;
    for (const Agent& agent : agents) {
        int end = paths.at(agent.id).back().t;
        schedule[agent.id] = TaskWindow{end, end};
    }
    return schedule;
}

}  // namespace

PrioritizedPlanner::PrioritizedPlanner(const Map& map, AStarConfig config, ReplanConfig replan_config)
    : map_(map), config_(config), replan_config_(replan_config) {}

bool PrioritizedPlanner::register_path(int agent_id, const Path& path) {
    // 장부가 굳는 시각(18장): 이 경로는 끝 시각부터 목적지에 계속 머물고(Tail), 이동 예약도 끝 시각
    // 전에서 끝나므로, 이 경로 때문에 장부가 바뀌는 건 끝 시각까지다. 등록이 중간에 거절돼도 이미
    // 적은 칸이 있을 수 있으므로 적기 전에 알린다.
    table_.mark_settled_from(path.back().t);
    // 경로의 모든 칸(vertex)과 모든 이동(edge)을 등록한다.
    //
    // vertex 등록에 reserve_if_unowned를 쓰는 이유: 이 경로는 A*가 이미
    // 등록된 점유를 피해서 찾은 것이지만, "시작 칸"만은 A*가 점유 검사를
    // 하지 않는다([04장] search_with_diagnostics는 다음 칸으로 이동할 때만
    // is_occupied를 확인한다). 만약 이 로봇의 시작 칸이 이미 더 높은
    // 순위 로봇이 정당하게 차지한 (칸,시각)과 같다면, 그건 실제 vertex
    // conflict다 — reserve()로 무조건 덮어쓰면 그 충돌이 조용히 사라지고
    // 이후 단계(Tail 검사 등)에서도 다시는 들키지 않는다.
    for (size_t i = 0; i < path.size(); ++i) {
        const SpaceTimeCell& cell = path[i];
        if (!table_.reserve_if_unowned(cell.x, cell.y, cell.t, agent_id)) {
            return false;
        }

        if (i + 1 < path.size()) {
            const SpaceTimeCell& next = path[i + 1];
            // edge도 등록 전에 검사한다: A*를 거친 경로는 이미 edge 검사를
            // 받았지만, replan()에서 옛 경로를 A* 없이 그대로 등록하는 로봇은
            // 앞 순서 로봇의 새 경로와 서로 자리를 맞바꾸는(swap) 충돌이 있어도
            // 여기서만 잡을 수 있다(reserve_edge는 양방향을 기록하므로
            // is_edge_occupied 한 번으로 반대 방향 이동을 확인할 수 있다).
            if (table_.is_edge_occupied(cell.x, cell.y, next.x, next.y, cell.t)) {
                return false;
            }
            table_.reserve_edge(cell.x, cell.y, next.x, next.y, cell.t);
        }
    }

    // Tail Reservation: 도착 시각부터 max_timestep까지, 목적지 칸을
    // 계속 이 로봇이 차지하고 있다고 가정해 추가로 예약한다.
    //
    // reserve_if_unowned를 쓰는 이유: 만약 이 구간의 어느 시각에 이미
    // 다른(더 높은 순위) 로봇이 정당하게 등록해둔 vertex가 있다면, 그건
    // 그 로봇이 그 시각에 거기 있다는 뜻이다. 이 로봇이 "도착 후 영원히
    // 머문다"고 주장하는 것과 정면으로 모순되므로 — 이는 조용히 넘어갈
    // 문제가 아니라 실제 충돌이다. 그래서 거절(false)이 한 번이라도
    // 생기면 전체를 실패로 보고한다.
    //
    // A*는 "도착 후 끝까지 목적지에 머물 수 있을 때만" 도착으로 인정하므로
    // ([04장]) A*가 방금 찾은 경로는 여기서 거절되지 않는다. 이 검사가 실제로
    // 충돌을 잡는 건 replan()에서 옛 경로를 A* 없이 그대로 등록할 때다.
    const SpaceTimeCell& last = path.back();
    for (int t = last.t; t <= config_.max_timestep; ++t) {
        if (!table_.reserve_if_unowned(last.x, last.y, t, agent_id)) {
            return false;
        }
    }
    return true;
}

std::optional<PlanResult> PrioritizedPlanner::plan(const std::vector<Agent>& agents) {
    std::unordered_set<int> seen_ids; // 혹시 모를 중복 (agent=로봇 id 중복 제거)
    for (const Agent& agent : agents) {
        if (!seen_ids.insert(agent.id).second) {
            throw std::invalid_argument("PrioritizedPlanner::plan: agents must not contain duplicate ids");
        }
        if (agent.service_time < 0) {
            throw std::invalid_argument("PrioritizedPlanner::plan: service_time must not be negative");
        }
    }

    table_.clear();

    PlanResult result;

    for (const Agent& agent : agents) {
        // 작업 시간을 넘기면 A*가 "도착 후 작업이 시간 한도 안에 끝나는지"까지 보고, 경로 끝에 작업
        // 칸을 붙여 준다(17장 결정 7). 작업 구간은 Tail Reservation(도착부터 max_timestep까지
        // 목적지 예약)에 이미 들어 있으므로, 다른 로봇이 작업 중인 목적지를 지나갈 일은 원래부터 없다.
        SpaceTimeAStar astar(map_, table_, config_);
        std::optional<Path> path =
            astar.search(agent.start, agent.goal, /*start_time=*/0, agent.service_time);

        if (!path.has_value()) {
            return std::nullopt;
        }

        if (!register_path(agent.id, *path)) {
            return std::nullopt;
        }
        result[agent.id] = *path;
    }

    return result;
}

Cell PrioritizedPlanner::position_at(const Path& path, int current_time) {
    // path[i].t는 i와 같은 값으로 단조 증가한다(path[0].t가 시작 시각).
    // current_time이 경로 길이를 넘으면 이미 도착해서 Tail 상태이므로
    // 목적지 칸(path.back())에 계속 있다고 본다(06장 6.4절 구현 디테일).
    int start_time = path.front().t;
    int index = current_time - start_time;
    if (index < 0) index = 0;
    if (static_cast<size_t>(index) >= path.size()) {
        return Cell{path.back().x, path.back().y};
    }
    return Cell{path[static_cast<size_t>(index)].x, path[static_cast<size_t>(index)].y};
}

bool PrioritizedPlanner::path_hits_obstacle(const Path& path, const std::vector<Cell>& new_obstacles,
                              int current_time) {
    // current_time 이후 시점만 본다 — 이미 지나간 과거는 다시 계획해도
    // 바꿀 수 없으므로 영향 판정에서 의미가 없다.
    int last_t = path.back().t;
    int check_until = std::max(last_t, current_time);
    for (int t = current_time; t <= check_until; ++t) {
        Cell here = position_at(path, t);
        for (const Cell& obstacle : new_obstacles) {
            if (here == obstacle) return true;
        }
    }
    return false;
}

void PrioritizedPlanner::reserve_new_obstacles(const std::vector<Cell>& new_obstacles, int current_time) {
    // new_obstacles는 current_time 이후로 영원히 막혀 있다고 가정한다
    // 06장은 "장애물이 생겼다"고만 말하고 언제 사라지는지는 다루지 않으므로,
    // max_timestep까지 계속 점유된 것으로 등록한다. agent_id를 생략해
    // owner=-1("주인 모름")로 기록한다 — 이건 로봇이 아니라 정적 장애물이므로
    // Tier 1의 get_owner 추적 대상에서 자연히 제외된다(06장 6.6절 3단계).
    // 장애물은 current_time부터 끝까지 그대로라, 장부를 바꾸는 건 current_time 한 번이다(18장).
    table_.mark_settled_from(current_time);
    for (const Cell& obstacle : new_obstacles) {
        for (int t = current_time; t <= config_.max_timestep; ++t) {
            table_.reserve(obstacle.x, obstacle.y, t);
        }
    }
}

bool PrioritizedPlanner::reserve_ongoing_tasks(const std::vector<Agent>& agents,
                                               const PlanResult& previous_paths,
                                               const TaskSchedule& schedule, int current_time) {
    for (const Agent& agent : agents) {
        const TaskWindow& window = schedule.at(agent.id);
        if (task_state_at(window, current_time) != TaskState::kWorking) continue;
        const Path& old_path = previous_paths.at(agent.id);
        // 이 표시는 작업 끝 시각까지만 있고 그다음 시각에 사라진다 — 사라지는 것도 장부가 바뀌는
        // 것이므로, 장부는 작업 끝 + 1부터 굳는다(18장).
        table_.mark_settled_from(window.end + 1);
        // 작업 중인 로봇은 옛 경로상 이 구간 내내 목적지에 있다. 장부는 "실제 위치"를 적는
        // 곳이므로 옛 경로의 칸을 그대로 적는다.
        for (int t = current_time; t <= window.end; ++t) {
            Cell here = position_at(old_path, t);
            if (!table_.reserve_if_unowned(here.x, here.y, t, agent.id)) return false;
        }
    }
    return true;
}

void PrioritizedPlanner::release_ongoing_task(const Agent& agent, const Path& old_path,
                                              const TaskWindow& window, int current_time) {
    if (task_state_at(window, current_time) != TaskState::kWorking) return;
    for (int t = current_time; t <= window.end; ++t) {
        Cell here = position_at(old_path, t);
        table_.unreserve_if_owned_by(here.x, here.y, t, agent.id);
    }
}

bool PrioritizedPlanner::try_yield_requests(const std::vector<Agent>& agents,
                                            const PlanResult& previous_paths,
                                            const TaskSchedule& schedule,
                                            const std::vector<Cell>& new_obstacles,
                                            int current_time, std::vector<int>& working_ids,
                                            PlanResult& paths, std::vector<int>& yielded_ids) {
    auto in_working = [&working_ids](int id) {
        return std::find(working_ids.begin(), working_ids.end(), id) != working_ids.end();
    };

    // 1. 늦어진 로봇마다, 그 로봇의 가장 짧은 길 위에 서 있는 고정 로봇을 후보로 모은다.
    //    table_은 방금 성공한 시도의 장부 그대로다(모든 로봇이 적혀 있음).
    std::vector<int> candidates;
    for (const Agent& agent : agents) {
        if (!in_working(agent.id)) continue;
        if (task_state_at(schedule.at(agent.id), current_time) != TaskState::kMoving) continue;

        Cell here = position_at(previous_paths.at(agent.id), current_time);
        const std::vector<Cell>& route = shortest_route(here, agent.goal, new_obstacles);
        if (route.empty()) continue;
        int fastest_arrival = current_time + static_cast<int>(route.size()) - 1;
        int arrival = paths.at(agent.id).back().t - agent.service_time;  // 작업 칸 앞 = 도착
        // 이미 가장 빠르거나, 문턱(yield_min_delay)보다 덜 늦었으면 건너뛴다.
        if (arrival - fastest_arrival < std::max(1, replan_config_.yield_min_delay)) continue;

        for (const SpaceTimeCell& cell :
             shortest_path_conflicts(here, agent.goal, new_obstacles, current_time)) {
            std::optional<int> owner = table_.get_owner(cell.x, cell.y, cell.t);
            if (!owner.has_value() || *owner == -1 || in_working(*owner)) continue;
            // 움직이는 로봇은 잠깐 기다리면 지나간다. 영원히 막는 건 서 있는 로봇(Tail)뿐이다.
            if (task_state_at(schedule.at(*owner), current_time) == TaskState::kMoving) continue;
            if (std::find(candidates.begin(), candidates.end(), *owner) == candidates.end()) {
                candidates.push_back(*owner);
            }
        }
    }
    if (candidates.empty()) return false;

    // 2. 후보를 명단 뒤에 붙여 한 번 더 계획한다(후보는 늦어진 로봇 뒤에 계획되어 그 길을 보고 비킨다).
    std::vector<int> extended = working_ids;
    extended.insert(extended.end(), candidates.begin(), candidates.end());
    std::optional<PlanResult> attempt = try_replan_set(agents, previous_paths, extended, schedule,
                                                       new_obstacles, current_time, nullptr, nullptr);
    if (!attempt.has_value()) return false;

    // 3. 판단 기준: 지금부터 로봇들이 자기 목적지가 아닌 칸에 있는 스텝 수의 합. 경로 비용 합(도착
    //    시각의 합)은 비켜 준 로봇의 경로가 돌아온 시각에 끝나서, 1스텝만 비켜도 크게 늘어난 것처럼
    //    보이므로 쓰지 않는다(19장 19.2).
    auto off_goal_steps = [&agents, current_time](const PlanResult& result) {
        long long total = 0;
        for (const Agent& agent : agents) {
            const Path& path = result.at(agent.id);
            for (int t = current_time; t <= path.back().t; ++t) {
                if (!(position_at(path, t) == agent.goal)) ++total;
            }
        }
        return total;
    };
    if (off_goal_steps(*attempt) >= off_goal_steps(paths)) return false;

    paths = std::move(*attempt);
    working_ids = std::move(extended);
    yielded_ids = std::move(candidates);
    return true;
}

std::optional<PlanResult> PrioritizedPlanner::try_replan_set(const std::vector<Agent>& agents,
                                              const PlanResult& previous_paths,
                                              const std::vector<int>& working_ids,
                                              const TaskSchedule& schedule,
                                              const std::vector<Cell>& new_obstacles,
                                              int current_time,
                                              std::vector<SpaceTimeCell>* out_blocked,
                                              std::optional<int>* out_blocked_owner) {
    table_.clear();
    reserve_new_obstacles(new_obstacles, current_time);
    if (!reserve_ongoing_tasks(agents, previous_paths, schedule, current_time)) return std::nullopt;

    PlanResult result;

    auto in_working = [&working_ids](int id) {
        return std::find(working_ids.begin(), working_ids.end(), id) != working_ids.end();
    };

    // 처리 순서를 정한다(12장). 아래 반복문의 내용은 두 방식이 똑같고, 순서만 다르다.
    std::vector<const Agent*> order;
    if (replan_config_.order == ReplanOrder::kFixedFirst) {
        // 1) 고정 로봇의 옛 경로를 먼저 전부 등록한다. 이 경로들은 이전 계획에서 서로
        //    충돌이 없었고 새 장애물과도 겹치지 않으므로(겹쳤다면 working이다) 거절되지
        //    않는다. 그래서 재계획 로봇은 이들을 모두 알고 피해서 경로를 찾는다.
        for (const Agent& agent : agents) {
            if (!in_working(agent.id)) order.push_back(&agent);
        }
        // 2) 재계획 로봇은 working_ids 순서 = 처음 영향받은 로봇(agents 순서) → 구조
        //    로봇(불려 온 순서). 구조 로봇은 자기가 막은 로봇이 이미 working_ids에 있을 때
        //    불려 오므로, 항상 그 로봇보다 뒤에 계획되어 그 로봇의 새 경로를 보고 피한다.
        for (int id : working_ids) {
            for (const Agent& agent : agents) {
                if (agent.id == id) {
                    order.push_back(&agent);
                    break;
                }
            }
        }
    } else {
        for (const Agent& agent : agents) order.push_back(&agent);
    }

    for (const Agent* agent_ptr : order) {
        const Agent& agent = *agent_ptr;
        bool is_working = in_working(agent.id);

        if (!is_working) {
            // working_ids에 없는 로봇: 기존 경로를 그대로 믿고 장벽으로 등록한다.(06장 6.4절 2단계)
            // 이 등록은 옛경로 그대로를 재등록한다는 의미이다.
            //
            // 이 등록이 거절될 수도 있다 — 이미 재탐색된 working_ids 로봇의
            // 새 경로(Tail 포함)가, 이 "안 건드린" 로봇의 옛 경로와 같은
            // (칸,시각)에서 겹치거나 서로 자리를 맞바꾸는 경우다(06장
            // 6.5절이 말하는 "K가 비켜주면 풀리는데 못 비키는" 상황의 한
            // 형태). 이때 거절된 칸의 현재 주인은 이미 working_ids에 있는
            // 로봇이라 get_owner로 추적해도 새 후보를 못 찾는다 — 추적해야
            // 할 것은 "거절 당한 로봇(agent.id) 자신"이다. 그래서 out_blocked
            // 대신 out_blocked_owner로 agent.id를 직접 알려준다.
            //
            // (ReplanOrder::kFixedFirst에서는 고정 로봇이 재계획 로봇보다 먼저 등록되므로
            // 이 거절이 원리상 생기지 않는다. previous_paths끼리 이미 충돌하는 잘못된
            // 입력일 때만 생기며, 그때도 같은 방식으로 그 로봇을 불러온다.)
            const Path& old_path = previous_paths.at(agent.id);
            if (!register_path(agent.id, old_path)) {
                if (out_blocked_owner != nullptr) *out_blocked_owner = agent.id;
                return std::nullopt;
            }
            result[agent.id] = old_path;
            continue;
        }

        // working_ids에 속한 로봇: 원래 영향받은 로봇이든, 에스컬레이션으로
        // 추가된 구조 로봇이든 구분하지 않고 전부 똑같이 처리한다.
        // 옛경로를 장벽으로 등록하지 않고, current_time 시점 위치에서 목적지까지
        // 새로 A* 재탐색한다(06장 6.6.1절). 구조 로봇도 이렇게 자유롭게 다시
        // 계산되어야만, 자기 옛 자리를 고집하지 않고 실제로 길을 비켜줄 수
        // 있다 — 옛 경로(특히 Tail Reservation)를 그대로 장벽으로 세워두면
        // "구조"라는 이름이 무색하게 아무 자리도 안 비켜주는 셈이 된다.
        //
        // 작업 중인 로봇은 작업이 끝나는 시각부터 다시 계획한다(17장 결정 6). 그 전까지는 옛 경로
        // 그대로(목적지에서 작업) 두고, 이어붙일 때도 그 시각을 기준으로 한다.
        const Path& old_path = previous_paths.at(agent.id);
        const TaskWindow& window = schedule.at(agent.id);
        ReplanStart start = replan_start(agent, window, current_time);
        Cell here = position_at(old_path, start.time);

        release_ongoing_task(agent, old_path, window, current_time);
        SpaceTimeAStar astar(map_, table_, config_);
        AStarResult astar_result =
            astar.search_with_diagnostics(here, agent.goal, start.time, start.service);

        if (!astar_result.path.has_value()) {
            if (out_blocked != nullptr) {
                *out_blocked = astar_result.blocked_attempts;
                // 13장: "막혀 본 모든 칸" 대신 "최단 경로와 부딪히는 칸"만 넘긴다. 단, 그중
                // 새로 불러올 수 있는(아직 재계획 중이 아닌) 로봇이 한 대도 없으면 원래
                // 방식 그대로 둔다 — 그렇지 않으면 Tier가 아무도 못 불러오고 끝나 버린다.
                if (replan_config_.rescue_selection == RescueSelection::kShortestPathConflicts) {
                    std::vector<SpaceTimeCell> conflicts =
                        shortest_path_conflicts(here, agent.goal, new_obstacles, start.time);
                    bool has_new_candidate = false;
                    for (const SpaceTimeCell& cell : conflicts) {
                        std::optional<int> owner = table_.get_owner(cell.x, cell.y, cell.t);
                        if (owner.has_value() && *owner != -1 && !in_working(*owner)) {
                            has_new_candidate = true;
                            break;
                        }
                    }
                    if (has_new_candidate) *out_blocked = std::move(conflicts);
                }
            }
            return std::nullopt;
        }

        // 옛 경로의 start.time 전 구간(과거, 그리고 작업 중이면 남은 작업)과 새로 찾은 경로를 이어붙인다.
        Path full_path = splice_paths(old_path, *astar_result.path, start.time);

        if (!register_path(agent.id, full_path)) return std::nullopt;
        result[agent.id] = full_path;
    }

    return result;
}

const std::vector<Cell>& PrioritizedPlanner::shortest_route(Cell from, Cell goal,
                                                           const std::vector<Cell>& new_obstacles) const {
    const auto index = [this](Cell c) {
        return static_cast<std::uint64_t>(c.y) * static_cast<std::uint64_t>(map_.width()) +
               static_cast<std::uint64_t>(c.x);
    };
    const std::uint64_t key = (index(from) << 32) | index(goal);
    auto it = route_memo_.find(key);
    if (it == route_memo_.end()) {
        ++route_computations_;
        it = route_memo_.emplace(key, static_shortest_path(map_, from, goal, new_obstacles)).first;
    }
    return it->second;
}

std::vector<SpaceTimeCell> PrioritizedPlanner::shortest_path_conflicts(Cell from, Cell goal,
                                                         const std::vector<Cell>& new_obstacles,
                                                         int current_time) const {
    std::vector<SpaceTimeCell> conflicts;
    const std::vector<Cell>& route = shortest_route(from, goal, new_obstacles);
    if (route.empty()) return conflicts;

    // route[i]에는 current_time + i에 있다고 본다(쉬지 않고 가는 가장 빠른 일정).
    for (size_t i = 0; i < route.size(); ++i) {
        int t = current_time + static_cast<int>(i);
        if (t > config_.max_timestep) break;
        const Cell& here = route[i];
        // 칸 충돌. 출발 칸(i=0)은 지금 서 있는 칸이라 검사하지 않는다(A*와 같은 규칙).
        if (i > 0 && table_.is_occupied(here.x, here.y, t)) {
            conflicts.push_back(SpaceTimeCell{here.x, here.y, t});
        }
        // 자리 맞바꾸기: here→next로 가는 t→t+1에 누가 next→here로 온다. 그 로봇은
        // t 시각에 next에 서 있으므로, 그 칸의 주인이 곧 상대 로봇이다.
        if (i + 1 < route.size()) {
            const Cell& next = route[i + 1];
            if (table_.is_edge_occupied(here.x, here.y, next.x, next.y, t)) {
                conflicts.push_back(SpaceTimeCell{next.x, next.y, t});
            }
        }
    }

    // 도착 후 머묾: 도착한 뒤 목적지를 차지하는 로봇이 있으면 끝까지 머물 수 없다.
    int arrival = current_time + static_cast<int>(route.size()) - 1;
    for (int t = arrival; t <= config_.max_timestep; ++t) {
        if (table_.is_occupied(goal.x, goal.y, t)) conflicts.push_back(SpaceTimeCell{goal.x, goal.y, t});
    }
    return conflicts;
}

std::optional<PlanResult> PrioritizedPlanner::full_replan(const std::vector<Agent>& agents,
                                           const PlanResult& previous_paths,
                                           const std::vector<Cell>& new_obstacles,
                                           int current_time) {
    require_zero_service(agents, "full_replan");
    return full_replan(agents, previous_paths, zero_service_schedule(agents, previous_paths),
                       new_obstacles, current_time);
}

std::optional<PlanResult> PrioritizedPlanner::full_replan(const std::vector<Agent>& agents,
                                           const PlanResult& previous_paths,
                                           const TaskSchedule& previous_schedule,
                                           const std::vector<Cell>& new_obstacles,
                                           int current_time) {
    table_.clear();
    reserve_new_obstacles(new_obstacles, current_time);
    // 모든 로봇을 다시 계획하지만, 작업 중인 로봇은 여기서도 작업이 끝날 때까지 안 움직인다.
    if (!reserve_ongoing_tasks(agents, previous_paths, previous_schedule, current_time)) {
        return std::nullopt;
    }

    PlanResult result;

    for (const Agent& agent : agents) {
        const Path& old_path = previous_paths.at(agent.id);
        const TaskWindow& window = previous_schedule.at(agent.id);
        ReplanStart start = replan_start(agent, window, current_time);
        Cell here = position_at(old_path, start.time);

        release_ongoing_task(agent, old_path, window, current_time);
        SpaceTimeAStar astar(map_, table_, config_);
        std::optional<Path> new_future = astar.search(here, agent.goal, start.time, start.service);
        if (!new_future.has_value()) return std::nullopt;

        Path full_path = splice_paths(old_path, *new_future, start.time);

        if (!register_path(agent.id, full_path)) return std::nullopt;
        result[agent.id] = full_path;
    }

    return result;
}

std::optional<ReplanResult> PrioritizedPlanner::replan(const std::vector<Agent>& agents,
                                         const PlanResult& previous_paths,
                                         const std::vector<Cell>& new_obstacles,
                                         int current_time) {
    require_zero_service(agents, "replan");
    return replan(agents, previous_paths, zero_service_schedule(agents, previous_paths),
                  new_obstacles, current_time);
}

std::optional<ReplanResult> PrioritizedPlanner::replan(const std::vector<Agent>& agents,
                                         const PlanResult& previous_paths,
                                         const TaskSchedule& previous_schedule,
                                         const std::vector<Cell>& new_obstacles,
                                         int current_time) {
    // 22장 메모장은 이 재계획 한 번 동안만 쓴다. 지난 재계획의 길은 새 장애물이 달랐을 수 있으므로 버린다.
    route_memo_.clear();
    route_computations_ = 0;

    // 1단계: 영향받는 로봇 판별 (06장 6.4절).
    std::vector<int> working_ids;
    for (const Agent& agent : agents) {
        const Path& old_path = previous_paths.at(agent.id);
        if (path_hits_obstacle(old_path, new_obstacles, current_time)) {
            working_ids.push_back(agent.id);

            // 도달성 검사(13장): 다른 로봇이 하나도 없어도 이 로봇이 목적지에 못 간다면
            // (새 장애물이 길을 끊었거나, 가장 빨라도 max_timestep 안에 못 도착한다면)
            // 어떤 로봇을 불러와도, 전체 재계획을 해도 이 로봇의 A*는 반드시 실패한다.
            // 그러니 Tier와 안전망을 헛돌지 않고 바로 실패를 알린다. 구조 로봇은 검사할
            // 필요가 없다 — 옛 경로가 새 장애물을 안 밟으므로 그 자체가 갈 수 있다는 증거다.
            // 작업 시간이 있으면 "가장 빨리 도착 + 작업"이 시간 안에 끝나야 한다(17장 결정 8). A*의
            // 도착 규칙과 똑같아야 "결과는 안 바꾸고 시간만 아낀다"가 유지된다.
            if (replan_config_.check_reachability) {
                ReplanStart start =
                    replan_start(agent, previous_schedule.at(agent.id), current_time);
                Cell here = position_at(old_path, start.time);
                const std::vector<Cell>& route = shortest_route(here, agent.goal, new_obstacles);
                if (route.empty()) return std::nullopt;
                int earliest_arrival = start.time + static_cast<int>(route.size()) - 1;
                if (earliest_arrival + start.service > config_.max_timestep) return std::nullopt;
            }
        }
    }

    // 영향받은 로봇이 아무도 없으면 기존 경로 그대로 끝낸다.
    if (working_ids.empty()) {
        ReplanResult result;
        result.paths = previous_paths;
        result.schedule = previous_schedule;
        result.escalation_tier = 0;
        return result;
    }

    // 2단계: Tier 0부터 max_escalation_tiers까지, 막은 원인 로봇을 추적해서
    // working_ids를 넓혀가며 재시도한다(06장 6.6절).
    std::vector<int> rescued_ids;
    for (int tier = 0; tier <= replan_config_.max_escalation_tiers; ++tier) {
        std::vector<SpaceTimeCell> blocked; // try_replan_set() -> out_blocked (working_ids에 포함된 로봇 경로계획 실패시)
        std::optional<int> blocked_owner;   // try_replan_set() -> out_blocked_owner (working_ids에 포함되지 않은 로봇 경로계획 실패시)
        std::optional<PlanResult> attempt = try_replan_set(
            agents, previous_paths, working_ids, previous_schedule, new_obstacles, current_time,
            &blocked, &blocked_owner);

        if (attempt.has_value()) {
            ReplanResult result;
            result.paths = *attempt;
            if (replan_config_.request_yield) {
                try_yield_requests(agents, previous_paths, previous_schedule, new_obstacles,
                                   current_time, working_ids, result.paths, result.yielded_ids);
            }
            result.replanned_ids = working_ids;
            result.escalation_tier = tier;
            result.rescued_ids = rescued_ids;
            result.schedule =
                update_schedule(agents, previous_schedule, result.paths, current_time);
            return result;
        }

        if (tier == replan_config_.max_escalation_tiers) break;

        bool added_any = false;

        // barrier 등록 자체가 거절된 경우: 거절당한 로봇 자신을 추가한다. (06장 6.6절 확장 — A*가 못 찾은 게 아니라 Tail 등록이 막힌 경우)
        // (working_ids에 포함되지 않은 로봇 경로계획 실패시)
        if (blocked_owner.has_value()) {
            bool already_working = std::find(working_ids.begin(), working_ids.end(),
                                              *blocked_owner) != working_ids.end();
            if (!already_working) {
                working_ids.push_back(*blocked_owner);
                rescued_ids.push_back(*blocked_owner);
                added_any = true;
            }
        }

        // A* 탐색 자체가 막힌 경우: 막힌 칸들의 주인을 추적해서 새"구조 로봇"을 working_ids에 추가한다(06장 6.6절 기본 메커니즘).
        // (working_ids에 포함된 로봇 경로계획 실패시)
        for (const SpaceTimeCell& cell : blocked) {
            std::optional<int> owner = table_.get_owner(cell.x, cell.y, cell.t);
            if (!owner.has_value() || *owner == -1) continue;

            bool already_working =
                std::find(working_ids.begin(), working_ids.end(), *owner) != working_ids.end();
            if (already_working) continue;

            working_ids.push_back(*owner);
            rescued_ids.push_back(*owner);
            added_any = true;
        }

        if (!added_any) break;
    }

    // 3단계: 안전망 — 전체 재계획(현재 위치 기준).
    std::optional<PlanResult> fallback =
        full_replan(agents, previous_paths, previous_schedule, new_obstacles, current_time);
    if (!fallback.has_value()) return std::nullopt;

    ReplanResult result;
    result.paths = *fallback;
    for (const Agent& agent : agents) result.replanned_ids.push_back(agent.id);
    result.escalation_tier = -1;
    result.schedule = update_schedule(agents, previous_schedule, result.paths, current_time);
    return result;
}

}  // namespace mapf
