// ─────────────────────────────────────────────────────────────────
// core/include/mapf/pbs.hpp
//
// 05_pbs.md를 코드로 옮긴 파일. (이번 단계는 plan()만 — replan은 06장/5단계)
//
// PBS(Priority-Based Search)의 핵심 규칙: 먼저 계획된 로봇의 경로는 절대
// 바뀌지 않는다. 나중에 계획되는 로봇이 그 경로를 피해서 자기 길을 찾는다.
// 우선순위는 agents 벡터의 순서 그대로다(02장 2.5절).
// ─────────────────────────────────────────────────────────────────
#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "mapf/agent.hpp"
#include "mapf/map.hpp"
#include "mapf/reservation_table.hpp"
#include "mapf/space_time_astar.hpp"

namespace mapf {

// 로봇id → 경로.
using PBSResult = std::unordered_map<int, Path>;

class PBS {
public:
    PBS(const Map& map, AStarConfig config = AStarConfig{});

    // agents를 순서대로(=우선순위 순으로) 전부 계획한다.
    // 한 로봇이라도 경로를 못 찾으면 전체가 실패(nullopt)한다.
    std::optional<PBSResult> plan(const std::vector<Agent>& agents);

private:
    // 경로를 테이블에 등록한다: 모든 (칸,시각)과 모든 이동을 등록하고,
    // 마지막으로 목적지 칸을 도착 시각부터 max_timestep까지 추가로
    // 예약한다(Tail Reservation, 05장 5.5절) — "도착 후에는 그 자리에
    // 영원히 머문다"고 가정해 미래 시각까지 미리 막아두는 것이다.
    void register_path(int agent_id, const Path& path);

    const Map& map_;
    AStarConfig config_;
    ReservationTable table_;
};

}  // namespace mapf
