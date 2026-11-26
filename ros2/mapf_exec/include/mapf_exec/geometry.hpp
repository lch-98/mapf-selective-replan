// mapf_exec/geometry.hpp — 칸↔미터 변환과 몸(footprint) 검사 (docs/31 31.3, 31.4).
//
// ROS를 모르는 순수 C++. 7-1의 파이썬 grid.py와 같은 약속을 쓴다:
//   - 격자: rows[y][x], (0,0) = 왼쪽 위. x = 열(오른쪽), y = 행(아래로).
//   - 월드: x 오른쪽(동), y 위(북). 원점 = 격자 왼쪽 아래 모서리.
//   - 칸 (열 x, 행 y)의 중심 = (x·cell + cell/2, (높이 − 1 − y)·cell + cell/2).
//
// 몸 모양(31.3): 실제 제원 사각형 앞뒤 0.178 m × 좌우 0.138 m, 사각형 중심이 회전 중심(바퀴 축)에서 0.032 m 뒤.
// 회전 원 반지름 ≈ 0.14 m → 칸 0.5 m 안에서 여유 0.11 m.
#pragma once

#include <array>
#include <vector>

namespace mapf_exec {

struct GridCell {
    int x{0};  // 열
    int y{0};  // 행
    bool operator==(const GridCell& o) const { return x == o.x && y == o.y; }
    bool operator!=(const GridCell& o) const { return !(*this == o); }
};

struct Vec2 {
    double x{0.0};
    double y{0.0};
};

struct Pose2 {
    double x{0.0};    // 회전 중심(바퀴 축 가운데), 월드 m
    double y{0.0};
    double yaw{0.0};  // 라디안, 0 = 월드 +x
};

// 칸 (열 x, 행 y)의 중심(월드 m). height = 격자 행 수.
Vec2 cell_center(GridCell c, int height, double cell);
// 월드 점이 들어 있는 칸. 격자 밖이어도 계산은 한다.
GridCell world_to_cell(Vec2 p, int height, double cell);
// 격자에서 (dx, dy)로 한 칸 갈 때 바라볼 방향(라디안). 격자 y는 아래로 커지므로 월드 방향은 (dx, −dy).
double move_yaw(int dx, int dy);

double wrap_angle(double a);

struct Footprint {
    double length{0.178};        // 앞뒤
    double width{0.138};         // 좌우
    double center_offset{-0.032};  // 사각형 중심이 회전 중심에서 앞(+)/뒤(−)로 얼마나
};

using Corners = std::array<Vec2, 4>;  // 볼록 사각형 꼭짓점(반시계)

// 자세 pose에서 몸 사각형의 네 꼭짓점(월드).
Corners footprint_corners(const Pose2& pose, const Footprint& fp = Footprint{});
// 회전 중심에서 몸의 가장 먼 점까지 거리 = 제자리 회전 때 쓸고 가는 원 반지름.
double sweep_radius(const Footprint& fp = Footprint{});

// 칸을 꽉 채우는 사각형(월드) — 선반, 또는 "가진 칸".
Corners cell_box(GridCell c, int height, double cell);

// 두 볼록 사각형이 겹치나(분리축 정리). 변끼리 닿기만 한 경우는 겹침 아님.
bool convex_overlap(const Corners& a, const Corners& b);
// 두 볼록 사각형 사이 최소 거리. 겹치면 0.
double convex_distance(const Corners& a, const Corners& b);

// 칸 이탈(31.4 (다)): 몸이 "가진 칸들"(1칸, 또는 이웃한 2칸) 밖으로 얼마나 나갔나(m). 0이면 안에 있음.
// 가진 칸이 이웃하지 않으면 std::invalid_argument.
double escape_depth(const Pose2& pose, const std::vector<GridCell>& held, int height, double cell,
                    const Footprint& fp = Footprint{});

}  // namespace mapf_exec
