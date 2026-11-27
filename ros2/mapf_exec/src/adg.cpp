#include "mapf_exec/adg.hpp"

#include <algorithm>
#include <stdexcept>

namespace mapf_exec {

Adg::Adg(const std::map<int, std::vector<GridCell>>& paths, const std::map<int, int>& service_steps,
         bool cross_edges, const std::map<int, int>& service_start) {
    for (const auto& [robot, path] : paths) {
        auto& list = by_robot_[robot];
        if (path.empty()) continue;
        // 이동 동작: 칸이 바뀌는 번호마다
        for (int t = 0; t + 1 < static_cast<int>(path.size()); ++t) {
            if (path[t] != path[t + 1]) {
                AdgAction a;
                a.robot = robot;
                a.index = static_cast<int>(list.size());
                a.kind = AdgAction::Kind::kMove;
                a.from = path[t];
                a.to = path[t + 1];
                a.plan_t = t;
                list.push_back(static_cast<int>(actions_.size()));
                actions_.push_back(a);
            }
        }
        // 작업 동작: service_start가 있으면 그 번호, 없으면 경로 끝 목적지에 처음 머물기 시작한 번호
        auto it = service_steps.find(robot);
        if (it != service_steps.end() && it->second > 0) {
            int arrive = static_cast<int>(path.size()) - 1;
            while (arrive > 0 && path[arrive - 1] == path.back()) --arrive;
            auto st = service_start.find(robot);
            if (st != service_start.end()) arrive = st->second;
            AdgAction a;
            a.robot = robot;
            a.kind = AdgAction::Kind::kService;
            a.from = a.to = path[std::min<std::size_t>(static_cast<std::size_t>(arrive), path.size() - 1)];
            a.plan_t = arrive;
            a.service_steps = it->second;
            // 그 번호 전에 시작하는 이동들 다음 자리에 끼워 넣고 번호(index)를 다시 매김
            std::size_t pos = 0;
            while (pos < list.size() && actions_[list[pos]].plan_t < arrive) ++pos;
            list.insert(list.begin() + static_cast<long>(pos), static_cast<int>(actions_.size()));
            actions_.push_back(a);
            for (std::size_t k = 0; k < list.size(); ++k) actions_[list[k]].index = static_cast<int>(k);
        }
    }

    cross_preds_.assign(actions_.size(), {});
    state_.assign(actions_.size(), 0);
    if (!cross_edges) return;
    for (int b = 0; b < static_cast<int>(actions_.size()); ++b) {
        const auto& B = actions_[b];
        if (B.kind != AdgAction::Kind::kMove) continue;
        for (int a = 0; a < static_cast<int>(actions_.size()); ++a) {
            const auto& A = actions_[a];
            if (A.robot == B.robot || A.kind != AdgAction::Kind::kMove) continue;
            if (A.from == B.to && A.plan_t <= B.plan_t) cross_preds_[b].push_back(a);
        }
    }
}

int Adg::num_cross_edges() const {
    int n = 0;
    for (const auto& p : cross_preds_) n += static_cast<int>(p.size());
    return n;
}

const std::vector<int>& Adg::robot_actions(int robot) const {
    static const std::vector<int> kEmpty;
    auto it = by_robot_.find(robot);
    return it == by_robot_.end() ? kEmpty : it->second;
}

std::vector<int> Adg::releasable() const {
    std::vector<int> out;
    for (const auto& [robot, list] : by_robot_) {
        // 그 로봇의 첫 "아직" 동작. 그 앞 동작이 하는 중이면 내줄 수 없음
        for (std::size_t k = 0; k < list.size(); ++k) {
            const int a = list[k];
            if (state_[a] == 2) continue;
            if (state_[a] == 1) break;  // 하는 중
            bool ok = true;
            for (int p : cross_preds_[a]) ok = ok && state_[p] == 2;
            if (ok) out.push_back(a);
            break;
        }
    }
    return out;
}

void Adg::release(int a) {
    if (state_.at(a) != 0) throw std::logic_error("Adg::release: 이미 내준 동작");
    state_[a] = 1;
}

void Adg::done(int a) {
    if (state_.at(a) != 1) throw std::logic_error("Adg::done: 내주지 않은 동작");
    state_[a] = 2;
}

bool Adg::finished() const {
    for (int s : state_) {
        if (s != 2) return false;
    }
    return true;
}

}  // namespace mapf_exec
