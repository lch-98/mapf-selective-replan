// ─────────────────────────────────────────────────────────────────
// core/src/pbs.cpp
//
// pbs.hpp에서 선언한 PBS::plan / PBS::replan의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/pbs.hpp"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

namespace mapf {

PBS::PBS(const Map& map, AStarConfig config, PBSConfig replan_config)
    : map_(map), config_(config), replan_config_(replan_config) {}

bool PBS::register_path(int agent_id, const Path& path) {
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
    const SpaceTimeCell& last = path.back();
    for (int t = last.t; t <= config_.max_timestep; ++t) {
        if (!table_.reserve_if_unowned(last.x, last.y, t, agent_id)) {
            return false;
        }
    }
    return true;
}

std::optional<PBSResult> PBS::plan(const std::vector<Agent>& agents) {
    std::unordered_set<int> seen_ids; // 혹시 모를 중복 (agent=로봇 id 중복 제거)
    for (const Agent& agent : agents) {
        if (!seen_ids.insert(agent.id).second) {
            throw std::invalid_argument("PBS::plan: agents must not contain duplicate ids");
        }
    }

    table_.clear();

    PBSResult result;

    for (const Agent& agent : agents) {
        SpaceTimeAStar astar(map_, table_, config_);
        std::optional<Path> path = astar.search(agent.start, agent.goal);

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

Cell PBS::position_at(const Path& path, int current_time) {
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

bool PBS::path_hits_obstacle(const Path& path, const std::vector<Cell>& new_obstacles,
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

void PBS::reserve_new_obstacles(const std::vector<Cell>& new_obstacles, int current_time) {
    // new_obstacles는 current_time 이후로 영원히 막혀 있다고 가정한다
    // 06장은 "장애물이 생겼다"고만 말하고 언제 사라지는지는 다루지 않으므로,
    // max_timestep까지 계속 점유된 것으로 등록한다. agent_id를 생략해
    // owner=-1("주인 모름")로 기록한다 — 이건 로봇이 아니라 정적 장애물이므로
    // Tier 1의 get_owner 추적 대상에서 자연히 제외된다(06장 6.6절 3단계).
    for (const Cell& obstacle : new_obstacles) {
        for (int t = current_time; t <= config_.max_timestep; ++t) {
            table_.reserve(obstacle.x, obstacle.y, t);
        }
    }
}

std::optional<PBSResult> PBS::try_replan_set(const std::vector<Agent>& agents,
                                              const PBSResult& previous_paths,
                                              const std::vector<int>& working_ids,
                                              const std::vector<Cell>& new_obstacles,
                                              int current_time,
                                              std::vector<SpaceTimeCell>* out_blocked,
                                              std::optional<int>* out_blocked_owner) {
    table_.clear();
    reserve_new_obstacles(new_obstacles, current_time);

    PBSResult result;

    for (const Agent& agent : agents) {
        bool is_working = std::find(working_ids.begin(), working_ids.end(), agent.id) !=
                           working_ids.end();

        if (!is_working) {
            // working_ids에 없는 로봇: 기존 경로를 그대로 믿고 장벽으로 등록한다.(06장 6.4절 2단계)
            // 이 등록은 옛경로 그대로를 재등록한다는 의미이다.
            //
            // 이 등록이 거절될 수도 있다 — 이미 재탐색된 working_ids 로봇의
            // 새 경로(Tail 포함)가, 이 "안 건드린" 로봇의 옛 Tail
            // Reservation과 정확히 그 (칸,시각)에서 충돌하는 경우다(06장
            // 6.5절이 말하는 "K가 비켜주면 풀리는데 못 비키는" 상황의 한
            // 형태). 이때 거절된 칸의 현재 주인은 이미 working_ids에 있는
            // 로봇이라 get_owner로 추적해도 새 후보를 못 찾는다 — 추적해야
            // 할 것은 "거절 당한 로봇(agent.id) 자신"이다. 그래서 out_blocked
            // 대신 out_blocked_owner로 agent.id를 직접 알려준다.
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
        const Path& old_path = previous_paths.at(agent.id);
        Cell here = position_at(old_path, current_time);

        SpaceTimeAStar astar(map_, table_, config_);
        AStarResult astar_result = astar.search_with_diagnostics(here, agent.goal, current_time);

        if (!astar_result.path.has_value()) {
            if (out_blocked != nullptr) *out_blocked = astar_result.blocked_attempts;
            return std::nullopt;
        }

        // 과거 구간(0~current_time-1)과 새로 찾은 미래 구간을 이어붙인다.
        Path full_path;
        for (const SpaceTimeCell& cell : old_path) {
            if (cell.t >= current_time) break;
            full_path.push_back(cell);
        }
        for (const SpaceTimeCell& cell : *astar_result.path) {
            full_path.push_back(cell);
        }

        if (!register_path(agent.id, full_path)) return std::nullopt;
        result[agent.id] = full_path;
    }

    return result;
}

std::optional<PBSResult> PBS::full_replan(const std::vector<Agent>& agents,
                                           const PBSResult& previous_paths,
                                           const std::vector<Cell>& new_obstacles,
                                           int current_time) {
    table_.clear();
    reserve_new_obstacles(new_obstacles, current_time);

    PBSResult result;

    for (const Agent& agent : agents) {
        const Path& old_path = previous_paths.at(agent.id);
        Cell here = position_at(old_path, current_time);

        SpaceTimeAStar astar(map_, table_, config_);
        std::optional<Path> new_future = astar.search(here, agent.goal, current_time);
        if (!new_future.has_value()) return std::nullopt;

        Path full_path;
        for (const SpaceTimeCell& cell : old_path) {
            if (cell.t >= current_time) break;
            full_path.push_back(cell);
        }
        for (const SpaceTimeCell& cell : *new_future) {
            full_path.push_back(cell);
        }

        if (!register_path(agent.id, full_path)) return std::nullopt;
        result[agent.id] = full_path;
    }

    return result;
}

std::optional<ReplanResult> PBS::replan(const std::vector<Agent>& agents,
                                         const PBSResult& previous_paths,
                                         const std::vector<Cell>& new_obstacles,
                                         int current_time) {
    // 1단계: 영향받는 로봇 판별 (06장 6.4절).
    std::vector<int> working_ids;
    for (const Agent& agent : agents) {
        const Path& old_path = previous_paths.at(agent.id);
        if (path_hits_obstacle(old_path, new_obstacles, current_time)) {
            working_ids.push_back(agent.id);
        }
    }

    // 영향받은 로봇이 아무도 없으면 기존 경로 그대로 끝낸다.
    if (working_ids.empty()) {
        ReplanResult result;
        result.paths = previous_paths;
        result.escalation_tier = 0;
        return result;
    }

    // 2단계: Tier 0부터 max_escalation_tiers까지, 막은 원인 로봇을 추적해서
    // working_ids를 넓혀가며 재시도한다(06장 6.6절).
    std::vector<int> rescued_ids;
    for (int tier = 0; tier <= replan_config_.max_escalation_tiers; ++tier) {
        std::vector<SpaceTimeCell> blocked; // try_replan_set() -> out_blocked (working_ids에 포함된 로봇 경로계획 실패시)
        std::optional<int> blocked_owner;   // try_replan_set() -> out_blocked_owner (working_ids에 포함되지 않은 로봇 경로계획 실패시)
        std::optional<PBSResult> attempt = try_replan_set(
            agents, previous_paths, working_ids, new_obstacles, current_time, &blocked,
            &blocked_owner);

        if (attempt.has_value()) {
            ReplanResult result;
            result.paths = *attempt;
            result.replanned_ids = working_ids;
            result.escalation_tier = tier;
            result.rescued_ids = rescued_ids;
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
    std::optional<PBSResult> fallback =
        full_replan(agents, previous_paths, new_obstacles, current_time);
    if (!fallback.has_value()) return std::nullopt;

    ReplanResult result;
    result.paths = *fallback;
    for (const Agent& agent : agents) result.replanned_ids.push_back(agent.id);
    result.escalation_tier = -1;
    return result;
}

}  // namespace mapf
