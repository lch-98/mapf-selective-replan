#include "mapf_exec/retime.hpp"

#include <deque>

namespace mapf_exec {

RetimeResult retime(const Adg& adg, const std::map<int, GridCell>& current,
                    const std::map<int, int>& inflight_service_steps) {
    const auto& acts = adg.actions();
    std::vector<int> done_time(acts.size(), -1);  // 새 번호로 끝난 시각(-1 = 아직). 이미 끝난 동작은 0
    std::map<int, std::deque<int>> queue;
    std::map<int, GridCell> pos;
    std::map<int, int> busy_until;  // 작업 중이면 이 번호까지 제자리
    RetimeResult out;

    for (const auto& [robot, cell] : current) {
        pos[robot] = cell;
        out.paths[robot] = {cell};
        // 지금 작업 중이면(실행기가 알려 줌) 그 스텝 동안 제자리 — 작업이 이 ADG의 동작이 아니어도
        // (앞선 재계획 전에 시작한 작업, 그 뒤 비켜 주는 이동만 이 ADG에 있는 경우) 똑같이 지킨다.
        auto inflight = inflight_service_steps.find(robot);
        busy_until[robot] = inflight == inflight_service_steps.end() ? 0 : inflight->second;
        if (busy_until[robot] > 0) {
            out.service_start[robot] = 0;
            out.service_steps[robot] = busy_until[robot];
        }
        for (int a : adg.robot_actions(robot)) {
            if (adg.is_done(a)) {
                done_time[a] = 0;
            } else if (adg.released(a)) {
                if (acts[a].kind == AdgAction::Kind::kMove) {
                    throw std::logic_error("retime: 이동 중인 로봇이 있음 — 이동을 마친 뒤 불러야 함");
                }
                // 하던 작업(이 ADG의 작업 동작): 남은 스텝 = 실행기가 알려 준 값, 없으면 계획 스텝 전부
                const int k = busy_until[robot] > 0 ? busy_until[robot] : acts[a].service_steps;
                busy_until[robot] = k;
                done_time[a] = k;
                out.service_start[robot] = 0;
                out.service_steps[robot] = k;
            } else {
                queue[robot].push_back(a);
            }
        }
    }

    int s = 0, idle = 0;
    auto remaining = [&] {
        for (const auto& [r, q] : queue) {
            if (!q.empty()) return true;
        }
        for (const auto& [r, b] : busy_until) {
            if (b > s) return true;
        }
        return false;
    };
    while (remaining()) {
        bool progress = false;
        for (auto& [robot, q] : queue) {
            if (busy_until[robot] > s || q.empty()) continue;
            const int a = q.front();
            const auto& A = acts[a];
            if (A.kind == AdgAction::Kind::kService) {
                busy_until[robot] = s + A.service_steps;
                done_time[a] = s + A.service_steps;
                out.service_start[robot] = s;
                out.service_steps[robot] = A.service_steps;
                q.pop_front();
                progress = true;
                continue;
            }
            bool ok = true;
            for (int p : adg.cross_preds(a)) ok = ok && done_time[p] >= 0 && done_time[p] <= s;
            if (!ok) continue;
            pos[robot] = A.to;
            done_time[a] = s + 1;
            q.pop_front();
            progress = true;
        }
        for (auto& [robot, path] : out.paths) path.push_back(pos[robot]);
        ++s;
        idle = progress ? 0 : idle + 1;
        bool someone_busy = false;
        for (const auto& [r, b] : busy_until) someone_busy = someone_busy || b > s;
        if (idle > 1000 && !someone_busy) throw std::logic_error("retime: 진행이 멈춤(약속에 고리가 있음?)");
    }
    return out;
}

}  // namespace mapf_exec
