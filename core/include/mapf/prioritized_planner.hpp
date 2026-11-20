// ─────────────────────────────────────────────────────────────────
// core/include/mapf/prioritized_planner.hpp
//
// 05_pbs.md(plan) + 06_selective_replan.md(replan)를 코드로 옮긴 파일.
//
// 고정 우선순위 계획(Prioritized Planning, PP)의 핵심 규칙: 먼저 계획된 로봇의 경로는
// 절대 바뀌지 않는다. 나중에 계획되는 로봇이 그 경로를 피해서 자기 길을 찾는다.
// 우선순위는 agents 벡터의 순서 그대로다(02장 2.5절).
//
// 이름 안내: 예전 이름은 PBS였다. 순서를 바꿔 가며 탐색하는 PBS(Ma et al., 2019)와
// 달리 순서를 고정하므로 PrioritizedPlanner로 바꿨다(10장 10.2절). 05장 문서의
// "PBS"는 이 클래스를 가리킨다.
// ─────────────────────────────────────────────────────────────────
#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "mapf/agent.hpp"
#include "mapf/map.hpp"
#include "mapf/reservation_table.hpp"
#include "mapf/space_time_astar.hpp"
#include "mapf/task_schedule.hpp"

namespace mapf {

// 로봇id → 경로.
using PlanResult = std::unordered_map<int, Path>;

// replan()의 한 번의 시도(try_replan_set)에서 로봇을 어떤 순서로 등록할지(12장).
enum class ReplanOrder {
    // agents 순서대로, 고정 로봇(옛 경로 등록)과 재계획 로봇(A*)을 섞어서 처리한다
    // (원래 방식). 앞 순서 재계획 로봇은 뒤 순서 고정 로봇의 옛 경로를 모른 채
    // 계획하므로, 고정 로봇의 등록이 거절될 수 있다(10장 10.5절 문제 1).
    kPriority,
    // 고정 로봇의 옛 경로를 먼저 전부 등록하고, 재계획 로봇은 working_ids 순서
    // (처음 영향받은 로봇 → 구조 로봇, 불려 온 순서)로 계획한다. 고정 로봇의 등록
    // 거절이 원리상 없고, 구조 로봇은 항상 자기가 막은 로봇보다 뒤에 계획된다
    // (11장 11.6절 아이디어 1). 대신 재계획 로봇은 자기보다 우선순위가 낮은 고정
    // 로봇까지 피해야 한다.
    kFixedFirst,
};

// 재계획 로봇의 A*가 실패했을 때, 누구를 구조 로봇으로 불러올지 고르는 방법(13장).
enum class RescueSelection {
    // A*가 탐색하다가 한 번이라도 막힌 모든 칸의 주인(원래 방식). A*는 실패하기까지
    // 갈 수 있는 곳을 전부 뒤지므로, 경로와 상관없는 먼 로봇까지 끌려올 수 있다.
    kBlockedCells,
    // "다른 로봇이 없을 때의 최단 경로"(지도의 벽 + 새 장애물만 보고 BFS)를 쉬지 않고
    // 따라간다고 보고, 그 경로와 실제로 부딪히는 로봇만 고른다(칸 충돌, 자리 맞바꾸기,
    // 도착 후 목적지 머묾). 그중 새로 불러올 로봇이 없으면 kBlockedCells로 돌아간다.
    kShortestPathConflicts,
};

// PrioritizedPlanner::replan의 동작을 조정하는 설정.
struct ReplanConfig {
    // Tier 1 이후, "막은 원인 로봇"을 몇 단계까지 추적해서 working_ids를
    // 넓혀볼지(06장 6.6절). 0이면 Tier 0만 시도하고 바로 안전망으로 간다.
    int max_escalation_tiers{1};

    // 한 번의 시도에서 로봇을 등록하는 순서. 안전망(full_replan)은 이 값과
    // 무관하게 항상 agents 순서다.
    ReplanOrder order{ReplanOrder::kPriority};

    // true면 replan()을 시작할 때, 영향받은 로봇마다 "다른 로봇이 하나도 없어도 목적지에
    // 갈 수 있는가"(벽 + 새 장애물만 보고 BFS, max_timestep 안에 도착 가능한가)를 확인한다.
    // 한 대라도 못 가면 Tier와 안전망을 건너뛰고 바로 실패(nullopt)한다(13장). 이때는 전체
    // 재계획도 반드시 실패하므로 결과는 같고, 헛된 탐색 시간만 아낀다.
    bool check_reachability{false};

    // A*가 실패했을 때 구조 로봇을 고르는 방법(13장).
    RescueSelection rescue_selection{RescueSelection::kBlockedCells};

    // true면 재계획이 성공한 뒤, 다시 계획한 로봇이 "로봇이 하나도 없을 때의 가장 빠른 도착"보다 늦게
    // 도착하게 됐을 때, 그 로봇의 가장 짧은 길 위에 서 있는(주차 중·작업 중) 고정 로봇에게 비켜 달라고
    // 해서 한 번 더 계획해 본다. 로봇들이 목적지 밖에 있는 시간의 합이 줄어들 때만 그 결과를 쓴다
    // (19장, 문제 3). 고정 먼저 방식에서는 서 있는 로봇을 피하는 먼 길이라도 있으면 A*가 성공해서
    // 비켜 달라고 할 기회가 없는데, 그걸 보완한다.
    bool request_yield{false};

    // request_yield일 때, 가장 빠른 도착보다 이 스텝 수 이상 늦어진 로봇이 있을 때만 시도한다. 한 번 더
    // 계획하는 데 시간이 들므로, 조금 늦은 경우는 건너뛰어 시간을 아낀다. 1이면 조금이라도 늦으면 시도.
    int yield_min_delay{1};

    // 6번 비교 기준 "Park식"(Park 외 2026, 27장 P1): 0 이상이면 Tier 0에서 "지금부터 affected_window 스텝
    // 안에 장애물 칸에 닿는 로봇"만 다시 계획한다. 더 늦게 닿는 로봇은 "미룬 로봇"(ReplanResult::deferred_ids)이
    // 되어 옛 경로를 그대로 두고, 장부에는 장애물에 닿기 바로 전 시각까지만 적는다(그 뒤는 아직 정해지지 않았으므로).
    // 미룬 로봇은 창 안에 들어올 때 다시 계획한다 — replan_windowed()가 그 시각마다 replan()을 다시 부른다.
    // -1(기본)이면 끔: 경로가 장애물과 겹치는 로봇을 전부 지금 다시 계획한다(원래 동작 그대로).
    int affected_window{-1};

    // 안전망 ②(30장, 2026-11-20): 0보다 크면, Tier와 안전망 ①(전체 재계획)까지 모두 실패했을 때만 "처음 영향받은 로봇들의
    // 순서"를 바꿔 Tier 0 → max_escalation_tiers를 이 횟수까지 다시 해 본다(고정 로봇은 그대로 고정, 선택성 유지).
    // PP는 순서에 따라 풀리고 안 풀리므로 순서만 바꿔도 풀리는 경우가 많다(PP 무작위 재시작, 알려진 기법).
    // 평소(Tier·전체 재계획이 성공하는 경우)에는 실행되지 않으므로 속도에 영향이 없다. 0(기본)이면 끔.
    int order_restarts{0};
    // 순서를 바꿀 때 쓰는 난수 씨앗. 같은 값이면 Windows·Ubuntu 어디서나 같은 순서가 나온다(결과 재현).
    std::uint32_t restart_seed{0};
};

// replan() 한 번의 결과 — 최종 경로 외에, "얼마나 선택적으로 풀렸는지"를
// 보여주는 디버깅·연구용 정보를 함께 담는다(06장 6.7절).
struct ReplanResult {
    PlanResult paths;                  // 최종 전체 경로(영향 안 받은 로봇은 기존 경로 그대로)
    std::vector<int> replanned_ids;    // 실제로 다시 탐색된 로봇 id 전부
    int escalation_tier{0};            // 0=Tier 0에서 끝남, 1+=그 단계까지 확장해서 성공, -1=전체 재계획 폴백
    std::vector<int> rescued_ids{};    // Tier 1+에서 새로 추가된 "구조 로봇" id 목록
    TaskSchedule schedule{};           // 재계획 후의 근무표(17장 결정 9, update_schedule로 계산)
    std::vector<int> yielded_ids{};    // 비켜 달라고 해서 결과에 쓰인 로봇 id(19장, request_yield일 때만)
    std::vector<int> deferred_ids{};   // affected_window 때문에 이번에 미룬 로봇 id(옛 경로가 아직 장애물을 밟음)
    std::vector<int> event_times{};
    int order_restart{0};              // 안전망 ②로 풀렸으면 몇 번째 순서 바꾸기에서 풀렸는지(1부터). 이때 escalation_tier는 -1    // replan_windowed()에서 replan()을 부른 시각들(첫 값 = current_time)
};

class PrioritizedPlanner {
public:
    PrioritizedPlanner(const Map& map, AStarConfig config = AStarConfig{}, ReplanConfig replan_config = ReplanConfig{});

    // agents를 순서대로(=우선순위 순으로) 전부 계획한다.
    // 한 로봇이라도 경로를 못 찾으면 전체가 실패(nullopt)한다.
    // agents에 중복된 id가 있으면 std::invalid_argument를 던진다 — 같은 id로
    // 등록되면 서로의 점유를 "내 것"으로 오인해 충돌을 못 잡고, result에서도
    // 한쪽 경로가 덮어써져 사라지는 잘못된 결과가 조용히 나온다.
    // agents가 빈 벡터이면 계획할 로봇이 없으므로 빈 PlanResult를 성공으로
    // 반환한다(실패가 아니다).
    //
    // 작업 시간(Agent::service_time, 17장): 각 경로는 도착 후 작업 칸까지 포함해서 끝난다.
    // 작업이 max_timestep 안에 안 끝나는 로봇이 있으면 실패한다. service_time이 음수면
    // std::invalid_argument를 던진다. 근무표는 schedule_from_plan(agents, 결과)로 만든다.
    std::optional<PlanResult> plan(const std::vector<Agent>& agents);

    // 이미 진행 중인 계획(previous_paths)에 new_obstacles가 새로 생겼을 때,
    // 영향받은 로봇만 선택적으로 다시 계획한다(06장). 기본(ReplanOrder::kPriority)
    // 에서는 agents 순서가 곧 우선순위라는 plan()의 규칙이 그대로 유지되고,
    // kFixedFirst에서는 "고정 로봇 → 처음 영향받은 로봇 → 구조 로봇" 순서가 된다.
    //
    // Tier 0(영향받은 로봇만) → Tier 1..max_escalation_tiers(막은 원인 로봇을
    // 추적해서 확장) → 안전망(전체 재계획, 현재 위치 기준)을 순서대로 시도한다.
    // 안전망까지도 실패하면 nullopt를 반환한다.
    //
    // 작업 시간(17장): previous_schedule은 previous_paths의 근무표다(처음 계획이면
    // schedule_from_plan, 재계획 결과면 그 ReplanResult::schedule). 지금 작업 중인 로봇은
    // 작업이 끝날 때까지 절대 움직이지 않고(구조 로봇으로 불려도 작업이 끝난 뒤에 비킨다),
    // 결과의 schedule에 재계획 후의 근무표를 담는다. previous_schedule에 없는 로봇이 있으면
    // std::out_of_range를 던진다.
    std::optional<ReplanResult> replan(const std::vector<Agent>& agents,
                                        const PlanResult& previous_paths,
                                        const TaskSchedule& previous_schedule,
                                        const std::vector<Cell>& new_obstacles,
                                        int current_time);

    // 근무표 없이 부르는 예전 버전. 모든 로봇의 작업 시간이 0일 때만 쓸 수 있다 — 이때는
    // "도착 = 작업 끝"이라 근무표를 경로에서 바로 만들 수 있다. 작업 시간이 있는 로봇이 있으면
    // 경로에서 근무표를 추측하면 조용히 틀릴 수 있으므로(17장 결정 3) std::invalid_argument를 던진다.
    std::optional<ReplanResult> replan(const std::vector<Agent>& agents,
                                        const PlanResult& previous_paths,
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
    //
    // 작업 중인 로봇은 여기서도 작업이 끝날 때까지 움직이지 않는다. 재계획 후 근무표는
    // update_schedule(agents, previous_schedule, 결과, current_time)로 만든다.
    // affected_window를 쓸 때의 재계획(6번 Park식): current_time에 replan()을 부르고, 미룬 로봇이 남아 있으면
    // 그중 가장 먼저 창에 들어오는 시각(장애물에 닿는 시각 - affected_window)으로 옮겨 다시 replan()을 부른다.
    // 미룬 로봇이 없어질 때까지 반복한다. 결과의 replanned_ids는 모든 회차의 합, escalation_tier는 한 번이라도
    // 안전망을 썼으면 -1, 아니면 회차 중 가장 큰 값, event_times는 부른 시각들. affected_window < 0이면 replan()과 같다.
    std::optional<ReplanResult> replan_windowed(const std::vector<Agent>& agents,
                                                const PlanResult& previous_paths,
                                                const TaskSchedule& previous_schedule,
                                                const std::vector<Cell>& new_obstacles,
                                                int current_time);

    // path가 current_time 이후 처음으로 new_obstacles 중 한 칸에 서는 시각. 없으면 nullopt.
    // path_hits_obstacle과 같은 범위(도착 후 머무는 시각 포함)를 본다.
    static std::optional<int> first_hit_time(const Path& path, const std::vector<Cell>& new_obstacles,
                                             int current_time);

    std::optional<PlanResult> full_replan(const std::vector<Agent>& agents,
                                          const PlanResult& previous_paths,
                                          const TaskSchedule& previous_schedule,
                                          const std::vector<Cell>& new_obstacles,
                                          int current_time);

    // 근무표 없이 부르는 예전 버전. replan의 예전 버전과 같은 규칙(작업 시간이 전부 0일 때만).
    std::optional<PlanResult> full_replan(const std::vector<Agent>& agents,
                                          const PlanResult& previous_paths,
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
    // 있다. PrioritizedPlanner 내부 판별 로직과 별도로 복제해서 만들면 나중에 한쪽만 바뀌어
    // 기준이 어긋날 위험이 있으므로, 실제로 쓰는 그 함수를 그대로 공유한다.
    static bool path_hits_obstacle(const Path& path, const std::vector<Cell>& new_obstacles,
                                    int current_time);

    // 마지막 replan()에서 "다른 로봇이 없을 때의 가장 짧은 길"(BFS)을 실제로 계산한 횟수(22장).
    // 같은 (출발 칸, 목적지)는 메모장에서 꺼내 쓰므로 세지 않는다. 테스트·확인용.
    int last_route_computations() const { return route_computations_; }

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
    // 미룬 로봇(affected_window)의 옛 경로를 until_t 전 시각까지만 등록한다(꼬리 예약 없음).
    bool register_path_prefix(int agent_id, const Path& path, int until_t);

    // current_time 시점에 agent_id가 있던 칸. path가 current_time보다
    // 짧으면(=이미 도착해서 Tail 상태) 목적지 칸을 반환한다(06장 6.4절).
    static Cell position_at(const Path& path, int current_time);

    // new_obstacles를 current_time부터 max_timestep까지 영원히 점유된 것으로
    // 테이블에 등록한다. owner를 지정하지 않아 -1("주인 모름")로 기록되므로,
    // Tier 1의 get_owner 추적에서 "구조 로봇 후보"로 잘못 잡히지 않는다.
    void reserve_new_obstacles(const std::vector<Cell>& new_obstacles, int current_time);

    // 17장 결정 5: 지금(current_time) 작업 중인 로봇마다, 남은 작업 칸(current_time ~ 작업 끝)을
    // 그 로봇 이름으로 장부에 먼저 적는다. 그래야 그 로봇이 구조 로봇이 되어 뒤 차례에 계획되더라도,
    // 앞 차례 로봇의 A*가 작업 중인 칸을 비어 있다고 보고 지나가지 않는다. 고정 로봇은 어차피 옛
    // 경로 전체를 적으므로 같은 칸을 한 번 더 적는 셈이고(자기 칸이라 문제없다), 작업 중인 칸에
    // 새 장애물이 생기는 등 이미 남이 차지한 칸이 있으면 false를 돌려준다.
    bool reserve_ongoing_tasks(const std::vector<Agent>& agents, const PlanResult& previous_paths,
                               const TaskSchedule& schedule, int current_time);

    // 19장(문제 3): replan이 성공한 직후 부른다. paths는 성공한 결과이고, working_ids는 그때의 재계획
    // 로봇 명단이다. 다시 계획한 "이동 중" 로봇이 가장 빠른 도착보다 늦게 도착하게 됐으면, 그 로봇의
    // 가장 짧은 길(쉬지 않고 가는 일정) 위에서 부딪히는 서 있는 고정 로봇을 명단 뒤에 붙여 한 번 더
    // 계획한다. 성공했고 "로봇들이 목적지 밖에 있는 시간의 합"이 줄었으면 paths·working_ids를 바꾸고
    // yielded_ids에 비켜 준 로봇을 담는다. 바꿨으면 true.
    bool try_yield_requests(const std::vector<Agent>& agents, const PlanResult& previous_paths,
                            const TaskSchedule& schedule, const std::vector<Cell>& new_obstacles,
                            int current_time, std::vector<int>& working_ids, PlanResult& paths,
                            std::vector<int>& yielded_ids);

    // 작업 중인 로봇이 자기 차례에 A*를 돌리기 직전에, 자기가 먼저 적어 둔 남은 작업 칸을 지운다.
    // 안 지우면 A*가 "목적지가 작업 끝 시각까지 예약돼 있다"고 보고(자기 예약인데도) 도착을 한
    // 스텝 늦게 인정한다. 지운 칸은 바로 뒤 register_path가 새 경로(남은 작업 칸 포함)로 다시 적는다.
    void release_ongoing_task(const Agent& agent, const Path& old_path, const TaskWindow& window,
                              int current_time);

    // working_ids에 속한 로봇만 current_time 시점의 위치에서 원래 목적지까지 다시
    // 탐색한다. working_ids에 없는 로봇은 previous_paths의 기존 경로를 그대로
    // 장벽으로 등록한다(06장 6.4절 2단계, 6.6.1절 "구조 로봇의 과거 경로를 장벽으로
    // 쓰면 안 된다"). 처리 순서는 replan_config_.order가 정한다(ReplanOrder 참고).
    //
    // 성공하면 새 PlanResult를 반환한다(영향 안 받은 로봇=기존 경로, working_ids
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
    std::optional<PlanResult> try_replan_set(const std::vector<Agent>& agents,
                                             const PlanResult& previous_paths,
                                             const std::vector<int>& working_ids,
                                             const TaskSchedule& schedule,
                                             const std::vector<Cell>& new_obstacles,
                                             int current_time,
                                             std::vector<SpaceTimeCell>* out_blocked,
                                             std::optional<int>* out_blocked_owner);

    // RescueSelection::kShortestPathConflicts가 쓰는 "구조 후보 칸" 목록. from에서 goal까지
    // 다른 로봇을 무시한 최단 경로를 current_time부터 쉬지 않고 따라간다고 볼 때, 지금
    // 테이블(table_)의 예약과 부딪히는 (칸, 시각)들을 돌려준다 — 칸 충돌은 그 칸, 자리
    // 맞바꾸기는 상대 로봇이 그 시각에 서 있는 칸, 도착 후 머묾은 목적지 칸. 호출하는 쪽이
    // get_owner로 주인을 찾는다. 최단 경로가 없으면 빈 목록.
    std::vector<SpaceTimeCell> shortest_path_conflicts(Cell from, Cell goal,
                                                        const std::vector<Cell>& new_obstacles,
                                                        int current_time) const;

    // from → goal의 "다른 로봇이 없을 때의 가장 짧은 길"(벽 + 새 장애물만 보는 BFS, 13장). 22장 메모장:
    // replan() 한 번 안에서 같은 (from, goal)은 처음 한 번만 계산하고 그 뒤로는 저장한 것을 돌려준다.
    // BFS는 다른 로봇·장부를 보지 않고, 새 장애물은 replan() 한 번 안에서 같으므로 결과가 반드시 같다.
    // 길(칸 목록)만 저장한다 — "그 길 위에 누가 있나"는 장부가 시도마다 바뀌므로 쓸 때마다 조회한다.
    // 못 가면 빈 목록.
    const std::vector<Cell>& shortest_route(Cell from, Cell goal,
                                            const std::vector<Cell>& new_obstacles) const;

    const Map& map_;
    AStarConfig config_;
    ReplanConfig replan_config_;
    ReservationTable table_;

    // 22장 메모장: 열쇠 = (출발 칸 번호 << 32) | 목적지 칸 번호. replan()을 시작할 때 비운다 — 다음 재계획은
    // 새 장애물이 다를 수 있다. shortest_path_conflicts(const)에서도 채우므로 mutable.
    // unordered_map은 원소를 더해도 기존 원소의 참조가 그대로라 shortest_route가 참조를 돌려줘도 안전하다.
    mutable std::unordered_map<std::uint64_t, std::vector<Cell>> route_memo_;
    mutable int route_computations_{0};
    // replan() 한 번 동안의 미룬 로봇: id → 장애물에 처음 닿는 시각(affected_window, 6번 Park식).
    std::unordered_map<int, int> deferred_hit_;
};

}  // namespace mapf
