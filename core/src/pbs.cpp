// ─────────────────────────────────────────────────────────────────
// core/src/pbs.cpp
//
// pbs.hpp에서 선언한 PBS::plan의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/pbs.hpp"

namespace mapf {

PBS::PBS(const Map& map, AStarConfig config) : map_(map), config_(config) {}

void PBS::reserve_all_goals(const std::vector<Agent>& agents) {
    for (const Agent& agent : agents) {
        for (int t = 0; t <= config_.max_timestep; ++t) {
            table_.reserve(agent.goal.x, agent.goal.y, t, agent.id);
        }
    }
}

void PBS::register_path(int agent_id, const Path& path) {
    // 경로의 모든 칸(vertex)과 모든 이동(edge)을 등록한다.
    for (size_t i = 0; i < path.size(); ++i) {
        const SpaceTimeCell& cell = path[i];
        table_.reserve(cell.x, cell.y, cell.t, agent_id);

        if (i + 1 < path.size()) {
            const SpaceTimeCell& next = path[i + 1];
            table_.reserve_edge(cell.x, cell.y, next.x, next.y, cell.t);
        }
    }

    // Tail Reservation: 도착 시각부터 max_timestep까지, 목적지 칸을
    // 계속 이 로봇이 차지하고 있다고 가정해 추가로 예약한다.
    const SpaceTimeCell& last = path.back();
    for (int t = last.t; t <= config_.max_timestep; ++t) {
        table_.reserve(last.x, last.y, t, agent_id);
    }
}

std::optional<PBSResult> PBS::plan(const std::vector<Agent>& agents) {
    table_.clear();
    reserve_all_goals(agents);

    PBSResult result;

    for (const Agent& agent : agents) {
        // 자기 차례가 되면, 자기 목적지의 임시 선점만 풀어준다 — 다른
        // 로봇들의 목적지 선점은 그대로 유지된다(05장 5.6절).
        //
        // unreserve_if_owned_by를 쓰는 이유: 만약 이미 다른 로봇이 이
        // 칸에 Tail Reservation(영구 점유, 05장 5.5절)을 걸어뒀다면,
        // 그건 이 agent_id의 것이 아니므로 절대 지우면 안 된다. 무조건
        // 지우는 unreserve()를 쓰면 그 보호가 깨진다.
        for (int t = 0; t <= config_.max_timestep; ++t) {
            table_.unreserve_if_owned_by(agent.goal.x, agent.goal.y, t, agent.id);
        }

        SpaceTimeAStar astar(map_, table_, config_);
        std::optional<Path> path = astar.search(agent.start, agent.goal);

        if (!path.has_value()) {
            return std::nullopt;
        }

        register_path(agent.id, *path);
        result[agent.id] = *path;
    }

    return result;
}

}  // namespace mapf
