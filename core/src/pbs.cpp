// ─────────────────────────────────────────────────────────────────
// core/src/pbs.cpp
//
// pbs.hpp에서 선언한 PBS::plan의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/pbs.hpp"

namespace mapf {

PBS::PBS(const Map& map, AStarConfig config) : map_(map), config_(config) {}

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

}  // namespace mapf
