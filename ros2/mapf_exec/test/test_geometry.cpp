// 손 계산 예시로 칸↔미터, 몸 사각형, 겹침, 칸 이탈을 확인한다.
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "mapf_exec/geometry.hpp"

using namespace mapf_exec;

TEST(Geometry, CellCenterFlipsRowsLikePythonGrid) {
    // ros2/mapf_gazebo/test/test_grid.py와 같은 값
    Vec2 a = cell_center({0, 0}, 10, 0.5);  // 0행 0열 = 맨 위 왼쪽
    EXPECT_DOUBLE_EQ(a.x, 0.25);
    EXPECT_DOUBLE_EQ(a.y, 4.75);
    Vec2 b = cell_center({0, 9}, 10, 0.5);  // 9행 0열 = 맨 아래 왼쪽
    EXPECT_DOUBLE_EQ(b.y, 0.25);
    Vec2 c = cell_center({3, 2}, 10, 0.5);  // 2행 3열
    EXPECT_DOUBLE_EQ(c.x, 1.75);
    EXPECT_DOUBLE_EQ(c.y, 3.75);
}

TEST(Geometry, WorldToCellRoundTrip) {
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 7; ++y) {
            Vec2 m = cell_center({x, y}, 7, 0.5);
            EXPECT_EQ(world_to_cell(m, 7, 0.5), (GridCell{x, y}));
            EXPECT_EQ(world_to_cell({m.x + 0.24, m.y - 0.24}, 7, 0.5), (GridCell{x, y}));
        }
    }
}

TEST(Geometry, MoveYaw) {
    EXPECT_DOUBLE_EQ(move_yaw(1, 0), 0.0);
    EXPECT_NEAR(move_yaw(0, -1), M_PI / 2, 1e-12);  // 위 행으로(y−1) = 북
    EXPECT_NEAR(move_yaw(0, 1), -M_PI / 2, 1e-12);
    EXPECT_NEAR(std::abs(move_yaw(-1, 0)), M_PI, 1e-12);
}

TEST(Geometry, FootprintCornersAtYawZero) {
    // 회전 중심 (0,0), 방향 0: 몸 x 범위 = −0.032 ± 0.089 = [−0.121, 0.057], y 범위 = ±0.069
    Corners c = footprint_corners({0.0, 0.0, 0.0});
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    for (auto& v : c) {
        xmin = std::min(xmin, v.x);
        xmax = std::max(xmax, v.x);
        ymin = std::min(ymin, v.y);
        ymax = std::max(ymax, v.y);
    }
    EXPECT_NEAR(xmin, -0.121, 1e-12);
    EXPECT_NEAR(xmax, 0.057, 1e-12);
    EXPECT_NEAR(ymin, -0.069, 1e-12);
    EXPECT_NEAR(ymax, 0.069, 1e-12);
}

TEST(Geometry, SweepRadiusLeavesMarginInHalfMeterCell) {
    // √(0.121² + 0.069²) = 0.13929
    EXPECT_NEAR(sweep_radius(), 0.139291, 1e-5);
    EXPECT_NEAR(0.25 - sweep_radius(), 0.1107, 1e-3);  // 칸 안 여유 0.11 m (31.3)
}

TEST(Geometry, NoEscapeWhenRotatingAtCellCenter) {
    const Vec2 m = cell_center({2, 3}, 10, 0.5);
    for (int deg = 0; deg < 360; deg += 5) {
        EXPECT_DOUBLE_EQ(escape_depth({m.x, m.y, deg * M_PI / 180.0}, {{2, 3}}, 10, 0.5), 0.0) << deg;
    }
}

TEST(Geometry, EscapeDepthWhenRearSticksOut) {
    // 1행 1열짜리 격자, 칸 중심 (0.25, 0.25). 중심에서 +x로 0.14 m 밀린 채 서쪽(180°)을 보면
    // 뒤쪽 끝이 +x로 0.14 + 0.121 = 0.261 → 칸 경계(0.25)보다 0.011 m 나감
    EXPECT_NEAR(escape_depth({0.39, 0.25, M_PI}, {{0, 0}}, 1, 0.5), 0.011, 1e-9);
    // 같은 자리에서 동쪽(0°)을 보면 앞쪽 끝 0.14 + 0.057 = 0.197 → 안 나감
    EXPECT_DOUBLE_EQ(escape_depth({0.39, 0.25, 0.0}, {{0, 0}}, 1, 0.5), 0.0);
}

TEST(Geometry, TwoHeldNeighborCellsFormOneRegion) {
    // 0행 0열과 0행 1열을 가진 채 두 칸 사이(경계 x = 0.5)를 지나는 중 → 안 나감
    EXPECT_DOUBLE_EQ(escape_depth({0.5, 0.25, 0.0}, {{0, 0}, {1, 0}}, 1, 0.5), 0.0);
    // 한 칸만 가졌다면 앞쪽이 0.057 m 나감
    EXPECT_NEAR(escape_depth({0.5, 0.25, 0.0}, {{0, 0}}, 1, 0.5), 0.057, 1e-9);
    EXPECT_THROW(escape_depth({0.5, 0.25, 0.0}, {{0, 0}, {2, 0}}, 1, 0.5), std::invalid_argument);
}

TEST(Geometry, BodyOverlapAndDistance) {
    Corners a = footprint_corners({0.0, 0.0, 0.0});  // x [−0.121, 0.057]
    // B가 +x로 0.2 m: x [0.079, 0.257] → 안 겹침, 틈 0.022
    Corners b = footprint_corners({0.2, 0.0, 0.0});
    EXPECT_FALSE(convex_overlap(a, b));
    EXPECT_NEAR(convex_distance(a, b), 0.022, 1e-9);
    // 0.17 m: x [0.049, 0.227] → 0.008 겹침
    Corners c = footprint_corners({0.17, 0.0, 0.0});
    EXPECT_TRUE(convex_overlap(a, c));
    EXPECT_DOUBLE_EQ(convex_distance(a, c), 0.0);
    // B가 0.2 m에서 북쪽(90°)을 봄: 몸 중심 (0.2, −0.032), x 범위 0.2 ± 0.069 = [0.131, 0.269] → 틈 0.074
    Corners d = footprint_corners({0.2, 0.0, M_PI / 2});
    EXPECT_FALSE(convex_overlap(a, d));
    EXPECT_NEAR(convex_distance(a, d), 0.074, 1e-9);
}

TEST(Geometry, TiltedBodyNearCornerNeedsBothShapesAxes) {
    // 칸 [0, 0.5]² 오른쪽 위 모서리 바깥에 45° 기운 마름모(중심 (0.57, 0.57), 중심→꼭짓점 0.1).
    // x·y축 그림자는 겹치지만(0.47 < 0.5), 마름모 자기 변의 법선 (1,1)/√2 방향으로는 떨어져 있다 → 겹침 아님.
    // 거리 = (2·0.57 − 0.1 − 1)/√2 = 0.04/√2 = 0.028284
    Corners box = cell_box({0, 0}, 1, 0.5);
    const double c = 0.57, r = 0.1;
    Corners diamond{Vec2{c + r, c}, Vec2{c, c + r}, Vec2{c - r, c}, Vec2{c, c - r}};
    EXPECT_FALSE(convex_overlap(box, diamond));
    EXPECT_FALSE(convex_overlap(diamond, box));
    EXPECT_NEAR(convex_distance(box, diamond), 0.04 / std::sqrt(2.0), 1e-9);
}

TEST(Geometry, TouchingIsNotOverlap) {
    Corners box = cell_box({0, 0}, 1, 0.5);  // [0, 0.5] × [0, 0.5]
    // 몸 앞쪽 끝을 x = 0에 딱 맞춤: 회전 중심 x = −0.057
    Corners body = footprint_corners({-0.057, 0.25, 0.0});
    EXPECT_FALSE(convex_overlap(box, body));
    EXPECT_NEAR(convex_distance(box, body), 0.0, 1e-12);
    Corners body_in = footprint_corners({-0.050, 0.25, 0.0});  // 7 mm 들어감
    EXPECT_TRUE(convex_overlap(box, body_in));
}
