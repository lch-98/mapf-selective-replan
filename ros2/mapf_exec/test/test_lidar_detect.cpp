// LiDAR 장애물 찾기(7-5): 손으로 만든 스캔으로 칸 판정과 확정 규칙을 확인한다.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "mapf_exec/lidar_detect.hpp"

using namespace mapf_exec;
using Kind = ObstacleConfirmer::Event::Kind;

namespace {
// 3행 × 4열, 1행 1열이 선반. 칸 0.5 m → 월드 y = (2 − 행) × 0.5 + 0.25
const std::vector<std::string> kRows = {"....", ".#..", "...."};
const double kCell = 0.5;

ScanData empty_scan() {
    ScanData s;
    s.angle_min = 0.0;
    s.angle_increment = 2.0 * M_PI / 360.0;
    s.ranges.assign(360, std::numeric_limits<float>::infinity());
    return s;
}
double deg(int k) { return k * M_PI / 180.0; }
int idx(int k) { return (k + 360) % 360; }

// 센서(sx, 0.25)에서 동쪽을 볼 때, 세로 면 x = face 에 닿는 광선(|y − 0.25| ≤ half)
ScanData wall_east(double sx, double face, double half, double noise = 0.0) {
    ScanData s = empty_scan();
    for (int k = -30; k <= 30; ++k) {
        const double a = deg(k);
        const double r = (face - sx) / std::cos(a);
        if (std::abs(r * std::sin(a)) <= half) s.ranges[idx(k)] = static_cast<float>(r + noise);
    }
    return s;
}
}  // namespace

TEST(LidarDetect, BoxInFreeCellIsFoundWithinDetectDistance) {
    // 로봇: 2행 0열 중심(0.25, 0.25), 동쪽. 센서 = 0.25 − 0.032 = 0.218. 상자(0.4 m): 2행 3열, 앞면 x = 1.5 + 0.05 = 1.55
    // 거리 1.332 m, 상자 반폭 0.2 → |각| ≤ atan(0.2/1.332) = 8.5° → −8°..8° 17점
    const ScanData s = wall_east(0.218, 1.55, 0.2);
    LidarParams p;
    auto cells = unknown_cells({0.25, 0.25, 0.0}, s, {}, kRows, kCell, p);
    ASSERT_EQ(cells.size(), 1u);
    EXPECT_EQ(cells.begin()->first, (std::pair<int, int>{3, 2}));
    EXPECT_EQ(cells.begin()->second, 17);
    p.d_detect = 1.0;  // 1.33 m > 1.0 m → 안 봄
    EXPECT_TRUE(unknown_cells({0.25, 0.25, 0.0}, s, {}, kRows, kCell, p).empty());
}

TEST(LidarDetect, RobotYawAndSensorOffsetAreUsed) {
    // 상자를 0행 3열(1.75, 1.25)에 두고, 로봇 2행 3열이 북쪽(yaw π/2)을 보면 광선 0° = 북. 앞면 y = 1.05, 센서 y = 0.25 − 0.032 = 0.218
    ScanData s = empty_scan();
    for (int k = -30; k <= 30; ++k) {
        const double a = deg(k), r = (1.05 - 0.218) / std::cos(a);
        if (std::abs(r * std::sin(a)) <= 0.2) s.ranges[idx(k)] = static_cast<float>(r);
    }
    auto cells = unknown_cells({1.75, 0.25, M_PI / 2}, s, {}, kRows, kCell, LidarParams{});
    ASSERT_EQ(cells.size(), 1u);
    EXPECT_EQ(cells.begin()->first, (std::pair<int, int>{3, 0}));  // 0행 3열
    // 센서 위치(회전 중심 0.032 m 뒤)를 써야 칸 경계 근처에서 맞는 칸이 나온다: 2행 0열에서 동쪽, 면 x = 1.44 → 밀면 1.49 = 2행 2열.
    // 회전 중심에서 쟀다고 잘못 보면 1.472 → 1.522 = 2행 3열
    auto near = unknown_cells({0.25, 0.25, 0.0}, wall_east(0.218, 1.44, 0.1), {}, kRows, kCell, LidarParams{});
    ASSERT_EQ(near.size(), 1u);
    EXPECT_EQ(near.begin()->first, (std::pair<int, int>{2, 2}));
}

TEST(LidarDetect, PointsOnOtherRobotsAreIgnored) {
    // 다른 로봇이 2행 2열(1.25, 0.25)에 있고 그 몸 앞면(x ≈ 1.11)에 점이 찍힘 → 회전 중심에서 0.14 m < 0.2 m → 버림
    const ScanData s = wall_east(0.218, 1.11, 0.069);
    EXPECT_TRUE(unknown_cells({0.25, 0.25, 0.0}, s, {{1.25, 0.25}}, kRows, kCell, LidarParams{}).empty());
    // 그 로봇 위치를 모르면 2행 2열을 장애물로 착각(그래서 필터가 필요)
    auto wrong = unknown_cells({0.25, 0.25, 0.0}, s, {}, kRows, kCell, LidarParams{});
    ASSERT_EQ(wrong.size(), 1u);
    EXPECT_EQ(wrong.begin()->first, (std::pair<int, int>{2, 2}));
}

TEST(LidarDetect, ShelfFaceAndWallWithNoiseAreNotFreeCells) {
    // 로봇 1행 0열(0.25, 0.75) 동쪽: 선반(1행 1열) 앞면 x = 0.5, 잡음으로 3 cm 짧게 → 점 x = 0.47(자기 칸).
    // 알고 있는 면에서 3 cm < 4 cm → 지도에 있는 물체로 보고 버림
    const ScanData s = wall_east(0.218, 0.5, 0.25, -0.03);
    LidarParams p;
    EXPECT_TRUE(unknown_cells({0.25, 0.75, 0.0}, s, {}, kRows, kCell, p).empty());
    p.map_clearance = 0.0;  // 면 거리를 안 보면 자기 칸(1행 0열)을 장애물로 착각(2 cm 밀어도 0.49)
    EXPECT_FALSE(unknown_cells({0.25, 0.75, 0.0}, s, {}, kRows, kCell, p).empty());
    // 선반 바로 아래 칸(2행 1열, 0.75, 0.25)에서 북쪽: 선반 아랫면 y = 0.5, 센서 y = 0.218, 3 cm 짧게 → 점 y ≈ 0.47(위 칸이 선반)
    ScanData up = empty_scan();
    for (int k = -20; k <= 20; ++k) up.ranges[idx(k)] = static_cast<float>(0.282 / std::cos(deg(k)) - 0.03);
    EXPECT_TRUE(unknown_cells({0.75, 0.25, M_PI / 2}, up, {}, kRows, kCell, LidarParams{}).empty());
    // 테두리 벽: 2행 0열에서 서쪽(yaw π) 벽 x = 0, 센서 x = 0.25 + 0.032 = 0.282, 3 cm 짧게 → 점 x ≈ 0.03(벽에서 4 cm 안) → 버림.
    // (벽 거리를 안 보면 2 cm 밀어도 0.01 = 자기 칸)
    ScanData w = empty_scan();
    for (int k = -20; k <= 20; ++k) w.ranges[idx(k)] = static_cast<float>(0.282 / std::cos(deg(k)) - 0.03);
    EXPECT_TRUE(unknown_cells({0.25, 0.25, M_PI}, w, {}, kRows, kCell, LidarParams{}).empty());
}

TEST(LidarDetect, FewPointsAreNotEnough) {
    ScanData s = empty_scan();
    s.ranges[0] = s.ranges[1] = static_cast<float>(1.55 - 0.218);  // 2점
    EXPECT_TRUE(unknown_cells({0.25, 0.25, 0.0}, s, {}, kRows, kCell, LidarParams{}).empty());
    s.ranges[359] = s.ranges[0];  // 3점
    EXPECT_EQ(unknown_cells({0.25, 0.25, 0.0}, s, {}, kRows, kCell, LidarParams{}).size(), 1u);
}

// ── 확정(미끄러지는 창): t_wait 3초 = 0.2초 칸 16개(처음 본 순간 포함), 90% → 15개 이상 ──
TEST(ObstacleConfirmer, ConfirmsWhenRecentWindowIsAtLeast90Percent) {
    for (int missing : {0, 1, 2}) {
        ObstacleConfirmer c(3.0);
        int confirmed_at = -1;
        for (int k = 0; k <= 20 && confirmed_at < 0; ++k) {
            if (k >= 1 && k <= missing) continue;  // 1~missing번째 스캔에서 못 봄
            c.observe(1.0 + 0.2 * k, {{3, 2}}, "robot_1");
            for (const auto& ev : c.update(1.0 + 0.2 * k)) {
                if (k == 0) {
                    EXPECT_EQ(ev.kind, Kind::kDetected);
                    EXPECT_DOUBLE_EQ(ev.t, 1.0);
                    EXPECT_EQ(ev.by, "robot_1");
                } else {
                    ASSERT_EQ(ev.kind, Kind::kConfirmed);
                    confirmed_at = k;
                    EXPECT_EQ(ev.bins_total, 16);
                    EXPECT_GE(ev.bins_seen, 15);
                }
            }
        }
        // 0~1번 놓치면 창이 처음 꽉 차는 15번째(3.0초 뒤)에 확정. 2번 놓치면 놓친 칸이 창에서 하나 빠지는 17번째에.
        EXPECT_EQ(confirmed_at, missing <= 1 ? 15 : 17) << missing;
        EXPECT_FALSE(c.is_candidate({3, 2}));
    }
}

// 실제로 겪은 장면(31.25): d_detect 경계에서 처음 보여 처음 2초 동안 8번 놓침(연달아 2번 넘게는 안 놓침), 그다음부터 계속 보임.
// t_wait 10초 = 51칸, 46칸 이상. 예전(고정 창)은 42/50으로 버렸다. 미끄러지는 창은 버리지 않고,
// 창 안의 놓친 칸이 5개 이하가 되는 55번째(창 5~55: 놓친 5·7·8·10·11)에 확정.
TEST(ObstacleConfirmer, FlickerAtDetectEdgeIsConfirmedLaterNotDropped) {
    const std::vector<int> miss = {1, 2, 4, 5, 7, 8, 10, 11};
    ObstacleConfirmer c(10.0);
    int confirmed_at = -1;
    for (int k = 0; k <= 70 && confirmed_at < 0; ++k) {
        if (std::find(miss.begin(), miss.end(), k) == miss.end()) c.observe(0.2 * k, {{5, 1}}, "robot_5");
        for (const auto& ev : c.update(0.2 * k)) {
            EXPECT_NE(ev.kind, Kind::kDropped) << k;
            if (ev.kind == Kind::kConfirmed) confirmed_at = k;
        }
    }
    EXPECT_EQ(confirmed_at, 55);
}

TEST(ObstacleConfirmer, ZeroWaitConfirmsAtOnceAndUnseenIsDroppedAfterOneSecond) {
    ObstacleConfirmer z(0.0);
    z.observe(2.0, {{1, 1}}, "robot_2");
    auto ev = z.update(2.0);
    ASSERT_EQ(ev.size(), 2u);
    EXPECT_EQ(ev[0].kind, Kind::kDetected);
    EXPECT_EQ(ev[1].kind, Kind::kConfirmed);

    ObstacleConfirmer c(3.0);
    c.observe(0.0, {{0, 0}}, "a");
    c.update(0.0);
    EXPECT_TRUE(c.update(1.0).empty());                 // 1초까지는 기다림
    EXPECT_EQ(c.update(1.2)[0].kind, Kind::kDropped);  // 1초 넘게 한 번도 안 보임 → 버림(치워진 것)
    c.observe(5.0, {{0, 0}}, "b");                       // 다시 보임 → 새 발견
    auto again = c.update(5.0);
    ASSERT_EQ(again.size(), 1u);
    EXPECT_EQ(again[0].kind, Kind::kDetected);
    EXPECT_DOUBLE_EQ(again[0].first_seen, 5.0);
    EXPECT_EQ(again[0].by, "b");
}

TEST(ObstacleConfirmer, HoldConfirmKeepsCandidate) {
    ObstacleConfirmer c(0.4);  // 3칸
    for (int k = 0; k < 3; ++k) c.observe(0.2 * k, {{0, 0}}, "a");
    c.set_hold_confirm(true);
    EXPECT_EQ(c.update(0.4).size(), 1u);  // 발견만
    EXPECT_TRUE(c.is_candidate({0, 0}));
    EXPECT_TRUE(c.update(3.0).empty());   // 확정 준비가 됐으면 안 보여도 버리지 않음(미룬 것뿐)
    c.set_hold_confirm(false);
    auto ev = c.update(3.1);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].kind, Kind::kConfirmed);
}

// 실제로 겪은 오탐(31.25): 선반 모서리 바로 위 옆면에 비스듬히 닿은 점들. 선반(1행 1열: x 0.5~1.0, y 0.5~1.0)의 오른쪽 면 x = 1.0을
// 1행 2열(1.25, 0.75)에서 북쪽을 보는 로봇이 왼쪽 아래(약 128°)로 봄 → 닿는 점이 모서리(1.0, 0.5)에서 0~3 cm 위.
// "광선 방향으로 5 cm 밀어 선반 안이면 버림"으로는 모서리를 지나 2행 1열(빈칸)로 빠졌다.
TEST(LidarDetect, GrazingHitsNearShelfCornerAreNotObstacles) {
    const Pose2 pose{1.25, 0.75, M_PI / 2};
    const double ox = 1.25, oy = 0.75 - 0.032;
    ScanData s = empty_scan();
    int near_corner = 0;
    for (int k = 100; k <= 160; ++k) {
        const double a = M_PI / 2 + deg(k);
        if (std::cos(a) >= 0) continue;
        const double r = (1.0 - ox) / std::cos(a), y = oy + r * std::sin(a);
        if (y < 0.5 || y > 1.0) continue;
        s.ranges[k] = static_cast<float>(r);
        if (y < 0.53) ++near_corner;
    }
    ASSERT_GE(near_corner, 3);  // 오탐을 만들 만큼 모서리 근처 점이 있음
    LidarParams p;
    EXPECT_TRUE(unknown_cells(pose, s, {}, kRows, kCell, p).empty());
    // 예전 규칙(면 거리 안 보고 5 cm 밀기)으로는 2행 1열이 나온다
    p.map_clearance = 0.0;
    p.push = 0.05;
    auto old = unknown_cells(pose, s, {}, kRows, kCell, p);
    ASSERT_EQ(old.size(), 1u);
    EXPECT_EQ(old.begin()->first, (std::pair<int, int>{1, 2}));
}

// 선반 바로 옆 칸의 상자도 찾는다: 2행 1열(선반 1행 1열의 아래) 상자(x 0.55~0.95, y 0.05~0.45)를 2행 3열에서 서쪽을 보며.
// 상자 위 모서리 점은 선반 면(y = 0.5)에서 5 cm, 바닥 벽(y = 0)에서 5 cm → 4 cm 기준에 안 걸림
TEST(LidarDetect, BoxNextToShelfIsStillFound) {
    ScanData s = empty_scan();
    const double sx = 1.75 + 0.032;
    int expected = 0;
    for (int k = -30; k <= 30; ++k) {
        const double a = M_PI + deg(k), r = (0.95 - sx) / std::cos(a), y = 0.25 + r * std::sin(a);
        if (y < 0.05 || y > 0.45) continue;
        s.ranges[idx(k)] = static_cast<float>(r);
        ++expected;
    }
    auto cells = unknown_cells({1.75, 0.25, M_PI}, s, {}, kRows, kCell, LidarParams{});
    ASSERT_EQ(cells.size(), 1u);
    EXPECT_EQ(cells.begin()->first, (std::pair<int, int>{1, 2}));
    EXPECT_EQ(cells.begin()->second, expected);
}
