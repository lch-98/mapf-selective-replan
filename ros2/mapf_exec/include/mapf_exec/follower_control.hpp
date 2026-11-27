// mapf_exec/follower_control.hpp — "돌고 → 직진 → 멈춤" 제어 계산 (docs/31 31.7).
//
// 한 번 부를 때마다 지금 자세와 구간(출발 칸 중심 → 목표 칸 중심)을 받아 선속도 v, 각속도 w를 낸다.
// ROS를 모르는 순수 함수라서 Gazebo 없이 단순 바퀴 모델로 적분해 시험할 수 있다(test_follower_control.cpp).
//
//   Rotate: 구간 방향과의 각도 차이가 heading_tol보다 크면 제자리 회전(직진 0).
//   Drive : 직진. 구간 방향 차이와 중심선에서 벗어난 거리(옆 오차)를 같이 줄이도록 회전을 섞는다.
//           남은 거리에 비례해 감속, 남은 거리(구간 방향으로 잰 것)가 stop_along 이하가 되면 Done.
//           방향 차이가 rotate_again보다 커지면 다시 Rotate(안전).
//   Done  : 멈춤.
// 칸 중심선 위로만 다니고 칸 중심에서만 돌아야 몸이 "가진 칸" 밖으로 안 나간다(31.4 (다)).
//
// 곧게 지나가기(31.24): carry > 0이면 목표 칸 다음에 같은 방향으로 carry(m)만큼 더 간다는 뜻이다(다음 동작을 이미 받음).
//   그때는 남은 거리 + carry로 감속을 계산하고(목표에서 서지 않음), 목표 중심을 지나는 순간(along <= 0) Done.
//   carry = 0이면 예전과 똑같다.
#pragma once

#include "mapf_exec/geometry.hpp"

namespace mapf_exec {

struct FollowerParams {
    double max_linear{0.15};      // m/s — 로봇별 값(시나리오 JSON의 max_linear)
    double max_angular{1.0};      // rad/s — 로봇별 값(max_angular)
    double heading_tol{0.0873};   // 5° — 이보다 작으면 직진 시작
    double rotate_again{0.5236};  // 30° — 직진 중 이보다 틀어지면 다시 제자리 회전
    double k_heading{2.0};        // 방향 차이 → 회전 속도
    double k_cross{4.0};          // 옆 오차(m) → 회전 속도
    double k_dist{1.2};           // 남은 거리(m) → 직진 속도(감속)
    double min_linear{0.03};      // 마지막에 너무 느려져 안 멈추는 것 방지
    double min_angular{0.15};     // 제자리 회전 최소 속도
    double stop_along{0.005};     // 구간 방향으로 남은 거리가 이 이하면 Done
};

enum class Phase { kRotate, kDrive, kDone };

struct Segment {
    Vec2 start;  // 출발 칸 중심
    Vec2 goal;   // 목표 칸 중심
};

struct ControlOutput {
    double v{0.0};
    double w{0.0};
    Phase phase{Phase::kDone};
};

// 구간 기준 지금 위치: along = 목표까지 구간 방향으로 남은 거리(지나치면 음수), cross = 중심선에서 왼쪽(+)/오른쪽(−)으로 벗어난 거리
struct SegmentError {
    double along{0.0};
    double cross{0.0};
    double heading{0.0};  // 구간 방향 − 로봇 방향 (wrap)
};
SegmentError segment_error(const Pose2& pose, const Segment& seg);

// start == goal(제자리)이면 바로 Done.
ControlOutput follow_step(const Pose2& pose, const Segment& seg, Phase phase, const FollowerParams& p, double carry = 0.0);

// next가 cur에 곧게 이어지나: next 출발 = cur 목표(tol 안), 두 구간 방향이 같음(1° 안). 길이 0인 구간은 아님.
bool continues_straight(const Segment& cur, const Segment& next, double tol = 0.01);
double segment_length(const Segment& s);

}  // namespace mapf_exec
