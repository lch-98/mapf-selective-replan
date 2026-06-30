// ─────────────────────────────────────────────────────────────────
// core/include/mapf/pbs.hpp
//
// 05_pbs.md(plan) + 06_selective_replan.md(replan)를 코드로 옮긴 파일.
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

// PBS::replan의 동작을 조정하는 설정.
struct PBSConfig {
    // Tier 1 이후, "막은 원인 로봇"을 몇 단계까지 추적해서 working_ids를
    // 넓혀볼지(06장 6.6절). 0이면 Tier 0만 시도하고 바로 안전망으로 간다.
    int max_escalation_tiers{1};
};

// replan() 한 번의 결과 — 최종 경로 외에, "얼마나 선택적으로 풀렸는지"를
// 보여주는 디버깅·연구용 정보를 함께 담는다(06장 6.7절).
struct ReplanResult {
    PBSResult paths;                  // 최종 전체 경로(영향 안 받은 로봇은 기존 경로 그대로)
    std::vector<int> replanned_ids;    // 실제로 다시 탐색된 로봇 id 전부
    int escalation_tier{0};            // 0=Tier 0에서 끝남, 1+=그 단계까지 확장해서 성공, -1=전체 재계획 폴백
    std::vector<int> rescued_ids{};    // Tier 1+에서 새로 추가된 "구조 로봇" id 목록
};

class PBS {
public:
    PBS(const Map& map, AStarConfig config = AStarConfig{}, PBSConfig replan_config = PBSConfig{});

    // agents를 순서대로(=우선순위 순으로) 전부 계획한다.
    // 한 로봇이라도 경로를 못 찾으면 전체가 실패(nullopt)한다.
    // agents에 중복된 id가 있으면 std::invalid_argument를 던진다 — 같은 id로
    // 등록되면 서로의 점유를 "내 것"으로 오인해 충돌을 못 잡고, result에서도
    // 한쪽 경로가 덮어써져 사라지는 잘못된 결과가 조용히 나온다.
    // agents가 빈 벡터이면 계획할 로봇이 없으므로 빈 PBSResult를 성공으로
    // 반환한다(실패가 아니다).
    std::optional<PBSResult> plan(const std::vector<Agent>& agents);

    // 이미 진행 중인 계획(previous_paths)에 new_obstacles가 새로 생겼을 때,
    // 영향받은 로봇만 선택적으로 다시 계획한다(06장). agents 순서가 곧
    // 우선순위라는 규칙은 plan()과 동일하게 유지된다.
    //
    // Tier 0(영향받은 로봇만) → Tier 1..max_escalation_tiers(막은 원인 로봇을
    // 추적해서 확장) → 안전망(전체 재계획, 현재 위치 기준)을 순서대로 시도한다.
    // 안전망까지도 실패하면 nullopt를 반환한다.
    std::optional<ReplanResult> replan(const std::vector<Agent>& agents,
                                        const PBSResult& previous_paths,
                                        const std::vector<Cell>& new_obstacles,
                                        int current_time);

    // "현재 위치 기준 전체 재계획" — replan()이 안전망으로 쓰는 것과 동일한
    // 로직(06장 6.7절)을 그대로 외부에 노출한다. working_ids 구분 없이
    // agents 전체를, previous_paths에서 current_time 시점에 있던 칸을 새
    // 출발점으로 삼아 처음부터 다시 계획한다(이미 지나온 길은 보존).
    //
    // 벤치마크에서 "전체 재계획 베이스라인"을 만들 때 쓴다 — replan()의
    // 선택적 재계획과 공정하게 비교하려면, 베이스라인도 "맨 처음 출발점"이
    // 아니라 "지금 로봇이 있는 위치"부터 다시 계획해야 하기 때문이다.
    std::optional<PBSResult> full_replan(const std::vector<Agent>& agents,
                                          const PBSResult& previous_paths,
                                          const std::vector<Cell>& new_obstacles,
                                          int current_time);

    // agent_id의 경로(path)가 current_time 이후 시점에 new_obstacles 중
    // 한 칸과 같은 (칸,시각)에서 겹치는지 확인한다(06장 6.4절 1단계) —
    // replan()이 "영향받는 로봇"을 판별할 때 쓰는 것과 정확히 같은 기준이다.
    //
    // public으로 노출하는 이유: 벤치마크 등 외부 코드가 "이 장애물이 실제로
    // 이 로봇의 기존 경로를 막는가"를 replan()/full_replan()을 호출하기
    // *전에* 직접 확인할 수 있어야 하기 때문이다 — 그래야 "장애물이 실제로는
    // 아무도 안 막는, 사실상 의미 없는 시나리오"를 비교 대상에서 걸러낼 수
    // 있다. PBS 내부 판별 로직과 별도로 복제해서 만들면 나중에 한쪽만 바뀌어
    // 기준이 어긋날 위험이 있으므로, 실제로 쓰는 그 함수를 그대로 공유한다.
    static bool path_hits_obstacle(const Path& path, const std::vector<Cell>& new_obstacles,
                                    int current_time);

private:
    // 경로를 테이블에 등록한다: 모든 (칸,시각)과 모든 이동을 등록하고,
    // 마지막으로 목적지 칸을 도착 시각부터 max_timestep까지 추가로
    // 예약한다(Tail Reservation, 05장 5.5절) — "도착 후에는 그 자리에
    // 영원히 머문다"고 가정해 미래 시각까지 미리 막아두는 것이다.
    //
    // 반환값: Tail Reservation 구간 전체를 문제없이 등록했으면 true.
    // 만약 그 구간의 어느 시각에 이미 다른(더 높은 순위) 로봇이 그 칸을
    // 점유 중이라면, 이 로봇은 도착 후 그 시각에 "영원히 머문다"는 전제
    // 자체가 거짓이 된다 — 즉 실제로는 풀 수 없는 충돌이다. 이 경우 false를
    // 반환하고, plan()은 전체를 실패로 처리한다.
    bool register_path(int agent_id, const Path& path);

    // current_time 시점에 agent_id가 있던 칸. path가 current_time보다
    // 짧으면(=이미 도착해서 Tail 상태) 목적지 칸을 반환한다(06장 6.4절).
    static Cell position_at(const Path& path, int current_time);

    // new_obstacles를 current_time부터 max_timestep까지 영원히 점유된 것으로
    // 테이블에 등록한다. owner를 지정하지 않아 -1("주인 모름")로 기록되므로,
    // Tier 1의 get_owner 추적에서 "구조 로봇 후보"로 잘못 잡히지 않는다.
    void reserve_new_obstacles(const std::vector<Cell>& new_obstacles, int current_time);

    // working_ids에 속한 로봇만 agents 순서대로(=우선순위 순으로) current_time
    // 시점의 위치에서 원래 목적지까지 다시 탐색한다. working_ids에 없는 로봇은
    // previous_paths의 기존 경로를 그대로 장벽으로 등록한다(06장 6.4절 2단계,
    // 6.6.1절 "구조 로봇의 과거 경로를 장벽으로 쓰면 안 된다").
    //
    // 성공하면 새 PBSResult를 반환한다(영향 안 받은 로봇=기존 경로, working_ids
    // 로봇=새로 찾은 경로를 과거 구간과 이어붙인 것). 실패하면 nullopt를 반환하고,
    // 실패 원인을 두 채널 중 하나로 알려준다(06장 6.6절의 get_owner 추적을
    // 가능하게 하는 정보):
    //
    //   - out_blocked: working_ids 로봇의 A* 탐색 자체가 막혔을 때, 그 막힌
    //     시공간 칸들(blocked_attempts). 호출하는 쪽이 get_owner로 누가
    //     막았는지 추적한다.
    //   - out_blocked_owner: working_ids에 "없는" 로봇을 기존 경로로 barrier
    //     등록하다가 거절당했을 때, 거절당한 그 로봇의 id 자신. 이 경우는
    //     "누가 막았는지"가 아니라 "이 로봇이 막혔다"는 사실 자체가 추적
    //     대상이다 — 거절된 칸의 현재 주인은 이미 working_ids에 있는 로봇이라
    //     get_owner로는 새 후보를 찾을 수 없기 때문이다.
    std::optional<PBSResult> try_replan_set(const std::vector<Agent>& agents,
                                             const PBSResult& previous_paths,
                                             const std::vector<int>& working_ids,
                                             const std::vector<Cell>& new_obstacles,
                                             int current_time,
                                             std::vector<SpaceTimeCell>* out_blocked,
                                             std::optional<int>* out_blocked_owner);

    const Map& map_;
    AStarConfig config_;
    PBSConfig replan_config_;
    ReservationTable table_;
};

}  // namespace mapf
