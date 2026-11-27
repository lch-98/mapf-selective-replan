// mapf_exec/adg.hpp — Action Dependency Graph (Hönig 외 2019, docs/31 31.4, 31.22).
//
// 1. 계획(로봇별 시간표) → 로봇별 "동작 목록". 칸이 바뀌는 순간만 이동 동작(kMove)이 되고, 중간의 제자리 대기는 버린다
//    (기다림은 약속이 만들어 준다). 경로 끝 목적지에서의 작업은 작업 동작(kService) 하나.
// 2. 약속(화살표):
//    - 같은 로봇: 동작 순서대로.
//    - 로봇 사이: 동작 a가 칸 c를 떠나고(a.from == c), 다른 로봇의 동작 b가 c로 들어가며(b.to == c),
//      b의 계획 시각이 a와 같거나 늦으면(a.plan_t <= b.plan_t) a → b. "b는 a가 끝난 뒤(= 앞 로봇이 다음 칸 중심에 도착한 뒤)에 출발".
// 3. 실행: 같은 로봇의 앞 동작이 끝났고, 그 로봇이 하는 동작이 없고, 들어오는 화살표가 모두 끝난 동작만 내준다.
// 계획이 충돌 없으면(plan_check 통과) 화살표는 늘 계획상 먼저 → 나중이라 고리가 없다 → 교착 없음.
#pragma once

#include <map>
#include <vector>

#include "mapf_exec/geometry.hpp"

namespace mapf_exec {

struct AdgAction {
    enum class Kind { kMove, kService };
    int robot{0};
    int index{0};  // 그 로봇의 몇 번째 동작인지(0부터)
    Kind kind{Kind::kMove};
    GridCell from{};
    GridCell to{};
    int plan_t{0};          // 계획에서 이 동작을 시작하는 번호(시각)
    int service_steps{0};   // kService: 계획에 넣은 작업 스텝
};

class Adg {
public:
    // paths: 로봇 → 시간표(i번째 = 번호 i의 칸). service_steps: 로봇 → 경로 끝 목적지에서의 작업 스텝(없으면 0).
    // cross_edges=false는 변이 검사용(로봇 사이 화살표를 모두 뺌). 실험에 쓰지 말 것.
    Adg(const std::map<int, std::vector<GridCell>>& paths, const std::map<int, int>& service_steps,
        bool cross_edges = true);

    const std::vector<AdgAction>& actions() const { return actions_; }
    // 동작 a로 들어오는 로봇 사이 화살표(앞 동작 번호들)
    const std::vector<int>& cross_preds(int a) const { return cross_preds_[a]; }
    int num_cross_edges() const;
    // 로봇의 동작 번호 목록(순서대로)
    const std::vector<int>& robot_actions(int robot) const;

    // ── 진행 ──
    std::vector<int> releasable() const;  // 지금 내줄 수 있는 동작들
    void release(int a);                  // 내줌(로봇이 하는 중)
    void done(int a);                     // 끝남
    bool released(int a) const { return state_[a] >= 1; }
    bool is_done(int a) const { return state_[a] == 2; }
    bool finished() const;

private:
    std::vector<AdgAction> actions_;
    std::vector<std::vector<int>> cross_preds_;
    std::map<int, std::vector<int>> by_robot_;
    std::vector<int> state_;  // 0 아직, 1 내줌, 2 끝남
};

}  // namespace mapf_exec
