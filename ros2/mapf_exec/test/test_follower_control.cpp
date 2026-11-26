// 돌고-가기 제어: 한 번 계산의 방향이 맞는지 + 단순 바퀴 모델로 끝까지 적분해 보기(Gazebo 없이).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mapf_exec/follower_control.hpp"

using namespace mapf_exec;

namespace {
const Segment kEast{{0.25, 0.25}, {0.75, 0.25}};  // 칸 중심에서 오른쪽(동) 이웃 칸 중심으로
}

TEST(FollowerControl, RotatesInPlaceFirstWhenFacingAway) {
    FollowerParams p;
    // 북쪽(90°)을 보고 있으면 오른쪽으로(음의 w) 제자리 회전
    auto out = follow_step({0.25, 0.25, M_PI / 2}, kEast, Phase::kRotate, p);
    EXPECT_EQ(out.phase, Phase::kRotate);
    EXPECT_DOUBLE_EQ(out.v, 0.0);
    EXPECT_LT(out.w, 0.0);
    EXPECT_LE(std::abs(out.w), p.max_angular);
}

TEST(FollowerControl, DrivesWhenAligned) {
    FollowerParams p;
    auto out = follow_step({0.25, 0.25, 0.0}, kEast, Phase::kRotate, p);
    EXPECT_EQ(out.phase, Phase::kDrive);
    EXPECT_GT(out.v, 0.0);
    EXPECT_LE(out.v, p.max_linear);
}

TEST(FollowerControl, SteersBackTowardCenterLine) {
    FollowerParams p;
    // 중심선보다 왼쪽(+y)으로 3 cm → 오른쪽으로(음의 w)
    auto out = follow_step({0.40, 0.28, 0.0}, kEast, Phase::kDrive, p);
    EXPECT_LT(out.w, 0.0);
}

TEST(FollowerControl, StopsAtGoalAndOnZeroSegment) {
    FollowerParams p;
    EXPECT_EQ(follow_step({0.749, 0.25, 0.0}, kEast, Phase::kDrive, p).phase, Phase::kDone);
    EXPECT_EQ(follow_step({0.80, 0.25, 0.0}, kEast, Phase::kDrive, p).phase, Phase::kDone);  // 지나침
    EXPECT_EQ(follow_step({0.25, 0.25, 0.0}, {{0.25, 0.25}, {0.25, 0.25}}, Phase::kRotate, p).phase, Phase::kDone);
}

TEST(FollowerControl, RotatesAgainIfHeadingDriftsTooMuch) {
    FollowerParams p;
    auto out = follow_step({0.40, 0.25, 0.8}, kEast, Phase::kDrive, p);  // 46° 틀어짐
    EXPECT_EQ(out.phase, Phase::kRotate);
    EXPECT_DOUBLE_EQ(out.v, 0.0);
}

// 단순 바퀴 모델(가속 제한 포함)로 끝까지 적분: 3 cm 안 도착, 옆으로 0.11 m 안, 크게 지나치지 않음
struct SimResult {
    double final_dist, max_cross, max_overshoot;
    int steps;
};

SimResult simulate(Pose2 pose, const Segment& seg, const FollowerParams& p) {
    const double dt = 0.05, accel = 0.5, ang_accel = 3.0;
    double v = 0.0, w = 0.0, max_cross = 0.0, max_over = 0.0;
    Phase ph = Phase::kRotate;
    int i = 0;
    for (; i < 2000; ++i) {
        auto out = follow_step(pose, seg, ph, p);
        ph = out.phase;
        if (ph == Phase::kDone) {
            v = w = 0.0;
            break;
        }
        v += std::clamp(out.v - v, -accel * dt, accel * dt);
        w += std::clamp(out.w - w, -ang_accel * dt, ang_accel * dt);
        pose.x += v * std::cos(pose.yaw) * dt;
        pose.y += v * std::sin(pose.yaw) * dt;
        pose.yaw = wrap_angle(pose.yaw + w * dt);
        auto e = segment_error(pose, seg);
        max_cross = std::max(max_cross, std::abs(e.cross));
        max_over = std::max(max_over, -e.along);
    }
    // 멈추라고 한 뒤 남은 속도로 미끄러지는 것까지 본다(가속 제한)
    return {std::hypot(seg.goal.x - pose.x, seg.goal.y - pose.y), max_cross, max_over, i};
}

TEST(FollowerControl, ClosedLoopReachesWithinThreeCentimeters) {
    for (auto [vmax, wmax] : {std::pair{0.12, 0.8}, std::pair{0.22, 1.5}}) {  // 31.13 D11 범위 양 끝
        FollowerParams p;
        p.max_linear = vmax;
        p.max_angular = wmax;
        for (double yaw0 : {0.0, M_PI / 2, -M_PI / 2, M_PI}) {  // 직진, 좌·우 90°, 뒤돌기
            // 출발 자세에 약간의 오차: 옆으로 2 cm, 방향 +3°
            SimResult r = simulate({0.25, 0.27, yaw0 + 0.05}, kEast, p);
            EXPECT_LT(r.final_dist, 0.03) << vmax << " " << yaw0;
            EXPECT_LT(r.max_cross, 0.11) << vmax << " " << yaw0;
            EXPECT_LT(r.max_overshoot, 0.03) << vmax << " " << yaw0;
            EXPECT_LT(r.steps, 2000);
        }
    }
}
