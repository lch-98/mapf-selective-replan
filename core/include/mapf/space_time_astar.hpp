// ─────────────────────────────────────────────────────────────────
// core/include/mapf/space_time_astar.hpp
//
// 04_space_time_astar.md를 코드로 옮긴 파일.
// "예약 테이블을 피해서, 로봇 한 대의 최단 경로를 찾는" 저수준 엔진.
// ─────────────────────────────────────────────────────────────────
#pragma once

#include <optional>
#include <vector>

#include "mapf/map.hpp"
#include "mapf/reservation_table.hpp"

namespace mapf {

// 시작부터 목적지까지의 시공간 경로. path[0]이 출발 시점에 대응한다.
using Path = std::vector<SpaceTimeCell>;

struct AStarConfig {
    // t가 이 값을 넘으면 그 가지(branch)를 포기한다 — 무한 탐색 방지.
    int max_timestep{256};
};

// 탐색 실패 시 "왜 막혔는지"까지 보고 싶을 때 쓰는 결과 타입.
// 06장(계층적 확장)에서 blocked_attempts의 각 칸을 ReservationTable::get_owner()로
// 조회해서 "누가 길을 막았는지"를 추적하는 데 쓰인다.
struct AStarResult {
    std::optional<Path> path;
    std::vector<SpaceTimeCell> blocked_attempts;
};

// 로봇 한 대의 경로를 찾는 시공간 A*. 이 엔진은 로봇이 몇 대 있는지,
// 왜 재계획을 하는지 전혀 모른다 — 그냥 주어진 장부를 피해서 최단 경로를
// 찾으라는 요청만 받고 응답한다.
class SpaceTimeAStar {
public:
    SpaceTimeAStar(const Map& map, const ReservationTable& reservations,
                   AStarConfig config = AStarConfig{});

    // 경로만 필요할 때. search_with_diagnostics()의 얇은 래퍼다.
    std::optional<Path> search(Cell start, Cell goal, int start_time = 0) const;

    // 실패 원인(막힌 시공간 칸들)까지 필요할 때.
    AStarResult search_with_diagnostics(Cell start, Cell goal, int start_time = 0) const;

private:
    const Map& map_;
    const ReservationTable& reservations_;
    AStarConfig config_;
};

}  // namespace mapf
