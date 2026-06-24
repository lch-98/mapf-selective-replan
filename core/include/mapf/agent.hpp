// ─────────────────────────────────────────────────────────────────
// core/include/mapf/agent.hpp
//
// 02_map_and_agent.md 2.4절을 코드로 옮긴 파일.
// ─────────────────────────────────────────────────────────────────
#pragma once

#include "mapf/map.hpp"

namespace mapf {

// 로봇 한 대. id, 출발 칸, 목적지 칸만 담는다.
//
// 우선순위 필드가 없는 이유: PBS(05장)에게 넘기는 std::vector<Agent>에서
// "앞에 있는 로봇일수록 우선순위가 높다"는 규칙으로 표현하기 때문이다.
// (02장 2.5절 참고)
struct Agent {
    int id{0};
    Cell start{};
    Cell goal{};
};

}  // namespace mapf
