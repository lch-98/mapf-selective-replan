// mapf_exec/plan_check.hpp — 계획 검사 (31.4 (가) 계획 안전).
//
// 코어(PrioritizedPlanner)와 **따로** 구현한 검사기다(tools/independent_check.py와 같은 규칙).
// 코어가 틀려도 여기서 잡히도록 코어 코드를 쓰지 않는다.
//
// 시간표: 로봇마다 칸 목록, 목록의 i번째 = 번호(시각) i의 칸. 목록이 끝나면 그 뒤로는 마지막 칸에 계속 있다고 본다
// (도착 후 목적지에 머무름 — 코어의 Tail 예약과 같은 뜻).
//
// 잡는 것
//   kEmptyPath   : 칸 목록이 비었음
//   kOffMap      : 격자 밖
//   kBlocked     : 선반('#') 칸
//   kJump        : 한 번호에 상하좌우 한 칸보다 많이 움직임(대각선 포함)
//   kVertex      : 같은 번호에 두 로봇이 같은 칸
//   kSwap        : 이웃한 두 로봇이 한 번호 동안 서로 자리를 바꿈
//   kObstacle    : 장애물이 생긴 번호부터 그 칸을 밟음
#pragma once

#include <map>
#include <string>
#include <vector>

#include "mapf_exec/geometry.hpp"

namespace mapf_exec {

enum class ViolationKind { kEmptyPath, kOffMap, kBlocked, kJump, kVertex, kSwap, kObstacle };

struct Violation {
    ViolationKind kind;
    int robot{-1};
    int other{-1};  // kVertex·kSwap의 상대 로봇
    int t{-1};      // 번호(시각)
    GridCell cell{};
    std::string describe() const;
};

struct TimedObstacle {
    GridCell cell{};
    int from_t{0};  // 이 번호부터 막힘
};

// rows: 격자(rows[y][x], '#' = 선반). paths: 로봇 번호 → 칸 목록(0번부터).
std::vector<Violation> check_plan(const std::vector<std::string>& rows,
                                  const std::map<int, std::vector<GridCell>>& paths,
                                  const std::vector<TimedObstacle>& obstacles = {});

}  // namespace mapf_exec
