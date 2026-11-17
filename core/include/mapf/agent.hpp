// ─────────────────────────────────────────────────────────────────
// core/include/mapf/agent.hpp
//
// 02_map_and_agent.md 2.4절을 코드로 옮긴 파일.
// ─────────────────────────────────────────────────────────────────
#pragma once

#include "mapf/map.hpp"

namespace mapf {

// 로봇 한 대. id, 출발 칸, 목적지 칸, 목적지에서 하는 작업의 시간을 담는다.
//
// 우선순위 필드가 없는 이유: PrioritizedPlanner(05장)에게 넘기는 std::vector<Agent>에서
// "앞에 있는 로봇일수록 우선순위가 높다"는 규칙으로 표현하기 때문이다.
// (02장 2.5절 참고)
struct Agent {
    int id{0};
    Cell start{};
    Cell goal{};

    // 목적지에 도착한 뒤 작업하는 스텝 수(17장 결정 1). 도착 시각이 t_a면 [t_a, t_a + service_time)
    // 동안 작업하고 그동안은 절대 움직이지 않는다. t_a + service_time부터는 주차 상태라, 목적지에
    // 머물지만 구조 로봇으로 불리면 비켜 줄 수 있다. 0이면 "도착 = 작업 완료"로 예전과 똑같다.
    int service_time{0};
};

}  // namespace mapf
