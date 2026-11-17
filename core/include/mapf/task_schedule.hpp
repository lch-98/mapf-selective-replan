// ─────────────────────────────────────────────────────────────────
// core/include/mapf/task_schedule.hpp
//
// 17_service_time_design.md 결정 3·9를 코드로 옮긴 파일.
// 로봇마다 "언제부터 언제까지 목적지에서 작업하는지"를 적은 근무표.
//
// 왜 경로와 따로 적나(17장 결정 3): 경로에는 "목적지에 머물러 있다"만 보이고, 그게 작업인지
// 주차인지는 안 보인다. 구조되어 비켰다가 돌아온 경로에서 "경로 끝 - 작업 시간"으로 계산하면
// 작업 시작이 틀리게 나온다. 그래서 도착하는 순간 정해진 작업 구간을 따로 기억한다.
// ─────────────────────────────────────────────────────────────────
#pragma once

#include <unordered_map>
#include <vector>

#include "mapf/agent.hpp"
#include "mapf/space_time_astar.hpp"

namespace mapf {

// 로봇 한 대의 작업 구간. 작업은 [start, end) 구간(스텝)에 하고, end 시각부터 움직일 수 있다.
// 예: t2에 도착, 작업 시간 3 → start=2, end=5. t2→t3, t3→t4, t4→t5에 작업하고 t5에 출발할 수 있다.
// 작업 시간이 0이면 start == end라서 "작업 중"인 순간이 없다.
struct TaskWindow {
    int start{0};  // 작업 시작 = 목적지 도착 시각
    int end{0};    // 작업 끝 = 이 시각부터 움직일 수 있다 (start + service_time)

    bool operator==(const TaskWindow& other) const {
        return start == other.start && end == other.end;
    }
};

// 로봇 id → 작업 구간.
using TaskSchedule = std::unordered_map<int, TaskWindow>;

// 어떤 시각 t에 로봇이 어떤 상태인지(17장 17.3절 표).
enum class TaskState {
    kMoving,   // t < start: 아직 도착 전. 재계획하면 도착 후 작업 시간만큼 머물러야 한다
    kWorking,  // start <= t < end: 작업 중. end까지는 절대 움직이지 않는다
    kParked,   // end <= t: 작업 끝, 주차 중. 목적지에 머물지만 불리면 비켜 줄 수 있다
};

TaskState task_state_at(const TaskWindow& window, int t);

// 경로 하나에서 작업 구간을 읽는다: end = 경로 끝 시각, start = end - service_time.
// 경로 끝의 (service_time + 1)개 칸이 전부 목적지여야 한다(A*가 작업 칸을 붙여 만든 경로).
// 아니면 std::invalid_argument를 던진다 — 작업 칸 없이 만든 경로나, 구조되어 비켰다가 돌아온
// 경로에서 작업 구간을 잘못 읽는 실수를 조용히 넘기지 않기 위해서다(17장 결정 3).
TaskWindow window_from_path(const Agent& agent, const Path& path);

// 처음 계획(plan)의 결과에서 근무표를 만든다. 로봇마다 window_from_path를 적용한다.
TaskSchedule schedule_from_plan(const std::vector<Agent>& agents,
                                const std::unordered_map<int, Path>& paths);

// 재계획 후의 근무표(17장 결정 9).
//   - current_time에 아직 도착 전(kMoving)이던 로봇: 새 경로로 다시 계산한다. 다시 계획됐다면
//     새 도착 시각이 나오고, 다시 계획되지 않았다면(고정 로봇) 경로가 같아서 값도 같다.
//   - 작업 중(kWorking)이거나 주차 중(kParked)이던 로봇: 이미 도착해서 작업 구간이 정해졌으므로
//     그대로 둔다. 구조되어 경로가 바뀌었어도 마찬가지다.
TaskSchedule update_schedule(const std::vector<Agent>& agents, const TaskSchedule& previous,
                             const std::unordered_map<int, Path>& new_paths, int current_time);

}  // namespace mapf
