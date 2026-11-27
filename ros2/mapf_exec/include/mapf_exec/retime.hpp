// mapf_exec/retime.hpp — 번호 다시 붙이기 (docs/31 31.6, 31.17).
//
// 재계획 직전, ADG로 실행 중이던 모습을 "지금 = 새 0번"인 시간표로 옮겨 적는다. 길(칸 순서)은 안 바꾸고 번호만 새로 붙인다.
// 규칙(31.17): 남은 동작을 한 스텝씩 흉내 낸다.
//   - 이동: 같은 로봇의 앞 동작이 끝났고, 로봇 사이 약속(앞 동작들)이 **이전 스텝까지** 모두 끝났으면 한 칸 이동, 아니면 제자리.
//   - 작업: 그 스텝 수만큼 제자리(하던 작업은 남은 스텝만).
//   - 이미 끝난 동작은 번호 0 이전에 끝난 것으로 본다.
// 결과 시간표는 충돌이 없고(같은 칸을 쓰는 두 로봇은 앞 로봇이 다 떠난 다음 스텝에 들어감), 칸 사용 순서가 옛 계획과 같다.
// 장애물 칸을 밟는 길도 그대로 옮겨 적는다 — 그 판단은 다음 단계의 replan()이 한다.
// 이 함수는 재계획 알고리즘이 아니다(통역). 우리 방법과 전체 재계획이 똑같이 거친다.
#pragma once

#include <map>
#include <stdexcept>
#include <vector>

#include "mapf_exec/adg.hpp"

namespace mapf_exec {

struct RetimeResult {
    std::map<int, std::vector<GridCell>> paths;  // 로봇 → 새 0번부터의 칸 목록
    std::map<int, int> service_start;            // 남은 작업이 시작되는 새 번호(남은 작업이 없으면 없음)
    std::map<int, int> service_steps;            // 그 작업 스텝 수
};

// adg: 진행 상태가 들어 있는 ADG. 이동 동작이 "하는 중"이면 안 됨(호출하는 쪽이 이동을 마칠 때까지 기다림) → std::logic_error.
// current: 로봇 → 지금 칸. inflight_service_steps: 지금 작업 중(작업 동작이 하는 중)인 로봇 → 남은 작업 스텝.
RetimeResult retime(const Adg& adg, const std::map<int, GridCell>& current,
                    const std::map<int, int>& inflight_service_steps);

}  // namespace mapf_exec
