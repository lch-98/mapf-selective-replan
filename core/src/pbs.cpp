// ─────────────────────────────────────────────────────────────────
// core/src/pbs.cpp
//
// pbs.hpp에서 선언한 PBS::plan의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/pbs.hpp"

namespace mapf {

PBS::PBS(const Map& map, AStarConfig config) : map_(map), config_(config) {}

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

    PBSResult result;

    for (const Agent& agent : agents) {
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
