// ─────────────────────────────────────────────────────────────────
// core/include/mapf/space_time_astar.hpp
//
// 04_space_time_astar.md를 코드로 옮긴 파일.
// "예약 테이블을 피해서, 로봇 한 대의 최단 경로를 찾는" 저수준 엔진.
// ─────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "mapf/map.hpp"
#include "mapf/reservation_table.hpp"

namespace mapf {

// 시작부터 목적지까지의 시공간 경로. path[0]이 출발 시점에 대응한다.
using Path = std::vector<SpaceTimeCell>;

// f가 같은 노드 중 무엇을 먼저 꺼낼지(동점 규칙, 18장 2-1 / 20장 원칙 3).
// 어느 규칙이든 A*가 찾는 경로의 길이(도착 시각)는 같다 — 길이가 같은 여러 경로 중 "어느 것"이
// 나오는지만 달라진다. 민감도 실험에서 결론이 이 선택에 흔들리지 않는지 보는 데 쓴다.
enum class TieBreak {
    kLargerG,   // 기본: g가 큰 쪽(목적지에 가까운 쪽) → 먼저 넣은 쪽
    kSmallerG,  // g가 작은 쪽 → 먼저 넣은 쪽
    kRandom,    // g가 큰 쪽 → tie_seed로 정한 무작위 번호 순(씨앗이 같으면 결과도 같다)
};

struct AStarConfig {
    // t가 이 값을 넘으면 그 가지(branch)를 포기한다 — 무한 탐색 방지.
    int max_timestep{256};

    // 동점 규칙(21장 5-5). 기본값이면 예전과 완전히 같다.
    TieBreak tie_break{TieBreak::kLargerG};
    // kRandom일 때 무작위 번호의 씨앗. 탐색(search 호출)마다 이 씨앗으로 새로 시작한다.
    std::uint32_t tie_seed{0};
};

// 탐색 실패 시 "왜 막혔는지"까지 보고 싶을 때 쓰는 결과 타입.
// 06장(계층적 확장)에서 blocked_attempts의 각 칸을 ReservationTable::get_owner()로
// 조회해서 "누가 길을 막았는지"를 추적하는 데 쓰인다.
struct AStarResult {
    std::optional<Path> path;
    std::vector<SpaceTimeCell> blocked_attempts;

    // 실제로 꺼내서 확인한 (칸, 시각) 상태 수(18장). 이미 확인한 상태라 건너뛴 것은 세지 않는다.
    // 탐색이 얼마나 일했는지 재고 테스트하는 데 쓴다.
    int expanded{0};
};

// 로봇 한 대의 경로를 찾는 시공간 A*. 이 엔진은 로봇이 몇 대 있는지,
// 왜 재계획을 하는지 전혀 모른다 — 그냥 주어진 장부를 피해서 최단 경로를
// 찾으라는 요청만 받고 응답한다.
//
// 도착 판정: goal에 도착한 뒤 max_timestep까지 goal이 한 번도 예약되어 있지
// 않아야 도착으로 인정한다(PrioritizedPlanner의 Tail Reservation과 같은 기준). 나중에 누가
// goal을 지나갈 예정이면 탐색을 계속해서, 비켜 있다가/기다렸다가 늦게 도착하는
// 경로를 찾는다. goal이 max_timestep까지 막혀 있으면 탐색 없이 바로 실패한다.
//
// 작업 시간(service_time, 17장 결정 7): 도착 후 그 스텝 수만큼 목적지에서 작업한다. 도착 시각이
// t_a면 t_a + service_time <= max_timestep이어야 도착으로 인정하고, 돌려주는 경로 끝에 목적지에
// 머무는 칸 service_time개((goal, t_a+1) ... (goal, t_a+service_time))를 붙인다. 작업 구간은
// 위의 "도착 후 max_timestep까지 goal이 비어 있어야 한다"에 이미 포함되므로 따로 확인할 것이
// 없다. service_time이 0이면 예전과 똑같다.
class SpaceTimeAStar {
public:
    SpaceTimeAStar(const Map& map, const ReservationTable& reservations,
                   AStarConfig config = AStarConfig{});

    // 경로만 필요할 때. search_with_diagnostics()의 얇은 래퍼다.
    std::optional<Path> search(Cell start, Cell goal, int start_time = 0,
                               int service_time = 0) const;

    // 실패 원인(막힌 시공간 칸들)까지 필요할 때.
    AStarResult search_with_diagnostics(Cell start, Cell goal, int start_time = 0,
                                        int service_time = 0) const;

private:
    const Map& map_;
    const ReservationTable& reservations_;
    AStarConfig config_;
};

}  // namespace mapf
