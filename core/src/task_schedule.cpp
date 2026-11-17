// ─────────────────────────────────────────────────────────────────
// core/src/task_schedule.cpp
//
// task_schedule.hpp에서 선언한 근무표 함수들의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/task_schedule.hpp"

#include <stdexcept>
#include <string>

namespace mapf {

TaskState task_state_at(const TaskWindow& window, int t) {
    if (t < window.start) return TaskState::kMoving;
    if (t < window.end) return TaskState::kWorking;
    return TaskState::kParked;
}

TaskWindow window_from_path(const Agent& agent, const Path& path) {
    const std::string who = "agent " + std::to_string(agent.id);
    if (agent.service_time < 0) {
        throw std::invalid_argument("window_from_path: " + who + " has negative service_time");
    }
    if (path.empty()) {
        throw std::invalid_argument("window_from_path: " + who + " has an empty path");
    }

    // 경로의 i번째 칸은 시각 path.front().t + i다. 작업 구간 [end - service_time, end]의 칸
    // (점으로 세면 service_time + 1개)이 경로 안에 있어야 하고, 전부 목적지여야 한다.
    const int stay_cells = agent.service_time + 1;
    if (static_cast<int>(path.size()) < stay_cells) {
        throw std::invalid_argument("window_from_path: " + who +
                                    "'s path is shorter than its service time");
    }
    for (size_t i = path.size() - static_cast<size_t>(stay_cells); i < path.size(); ++i) {
        if (path[i].x != agent.goal.x || path[i].y != agent.goal.y) {
            throw std::invalid_argument(
                "window_from_path: " + who + "'s path does not stay at its goal for the last " +
                std::to_string(agent.service_time) + " steps (not a freshly planned path?)");
        }
    }

    TaskWindow window;
    window.end = path.back().t;
    window.start = window.end - agent.service_time;
    return window;
}

TaskSchedule schedule_from_plan(const std::vector<Agent>& agents,
                                const std::unordered_map<int, Path>& paths) {
    TaskSchedule schedule;
    for (const Agent& agent : agents) {
        schedule[agent.id] = window_from_path(agent, paths.at(agent.id));
    }
    return schedule;
}

TaskSchedule update_schedule(const std::vector<Agent>& agents, const TaskSchedule& previous,
                             const std::unordered_map<int, Path>& new_paths, int current_time) {
    TaskSchedule schedule;
    for (const Agent& agent : agents) {
        const TaskWindow& old_window = previous.at(agent.id);
        if (task_state_at(old_window, current_time) == TaskState::kMoving) {
            schedule[agent.id] = window_from_path(agent, new_paths.at(agent.id));
        } else {
            schedule[agent.id] = old_window;
        }
    }
    return schedule;
}

}  // namespace mapf
