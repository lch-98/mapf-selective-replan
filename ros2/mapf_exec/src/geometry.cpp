#include "mapf_exec/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mapf_exec {

Vec2 cell_center(GridCell c, int height, double cell) {
    return {c.x * cell + cell / 2.0, (height - 1 - c.y) * cell + cell / 2.0};
}

GridCell world_to_cell(Vec2 p, int height, double cell) {
    return {static_cast<int>(std::floor(p.x / cell)), height - 1 - static_cast<int>(std::floor(p.y / cell))};
}

double move_yaw(int dx, int dy) { return std::atan2(-static_cast<double>(dy), static_cast<double>(dx)); }

double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

Corners footprint_corners(const Pose2& pose, const Footprint& fp) {
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const double hl = fp.length / 2.0, hw = fp.width / 2.0;
    // 몸 기준 좌표(앞 = +x, 왼쪽 = +y)의 네 꼭짓점을 반시계로
    const double local[4][2] = {{fp.center_offset + hl, -hw}, {fp.center_offset + hl, hw},
                                {fp.center_offset - hl, hw}, {fp.center_offset - hl, -hw}};
    Corners out{};
    for (int i = 0; i < 4; ++i) {
        out[i] = {pose.x + c * local[i][0] - s * local[i][1], pose.y + s * local[i][0] + c * local[i][1]};
    }
    return out;
}

double sweep_radius(const Footprint& fp) {
    const double far_x = std::abs(fp.center_offset) + fp.length / 2.0;
    return std::hypot(far_x, fp.width / 2.0);
}

Corners cell_box(GridCell c, int height, double cell) {
    const Vec2 m = cell_center(c, height, cell);
    const double h = cell / 2.0;
    return {Vec2{m.x - h, m.y - h}, Vec2{m.x + h, m.y - h}, Vec2{m.x + h, m.y + h}, Vec2{m.x - h, m.y + h}};
}

namespace {

// 축 (ax, ay)에 사각형을 투영한 [min, max]
void project(const Corners& p, double ax, double ay, double& lo, double& hi) {
    lo = std::numeric_limits<double>::infinity();
    hi = -lo;
    for (const auto& v : p) {
        const double d = v.x * ax + v.y * ay;
        lo = std::min(lo, d);
        hi = std::max(hi, d);
    }
}

bool separated_on_edges(const Corners& a, const Corners& b) {
    for (int i = 0; i < 4; ++i) {
        const Vec2& p = a[i];
        const Vec2& q = a[(i + 1) % 4];
        const double ax = -(q.y - p.y), ay = q.x - p.x;  // 변의 법선
        double alo, ahi, blo, bhi;
        project(a, ax, ay, alo, ahi);
        project(b, ax, ay, blo, bhi);
        // 닿기만 한 것(ahi == blo)은 겹침 아님 → 작은 허용치
        const double eps = 1e-12 * (std::abs(ax) + std::abs(ay));
        if (ahi <= blo + eps || bhi <= alo + eps) return true;
    }
    return false;
}

double point_segment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

}  // namespace

bool convex_overlap(const Corners& a, const Corners& b) {
    return !separated_on_edges(a, b) && !separated_on_edges(b, a);
}

double convex_distance(const Corners& a, const Corners& b) {
    if (convex_overlap(a, b)) return 0.0;
    // 겹치지 않는 두 볼록 다각형의 최소 거리 = 꼭짓점–변 거리 중 최소
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            best = std::min(best, point_segment(a[i], b[j], b[(j + 1) % 4]));
            best = std::min(best, point_segment(b[i], a[j], a[(j + 1) % 4]));
        }
    }
    return best;
}

double escape_depth(const Pose2& pose, const std::vector<GridCell>& held, int height, double cell,
                    const Footprint& fp) {
    if (held.empty() || held.size() > 2) throw std::invalid_argument("escape_depth: 가진 칸은 1칸 또는 2칸");
    if (held.size() == 2 && std::abs(held[0].x - held[1].x) + std::abs(held[0].y - held[1].y) > 1) {
        throw std::invalid_argument("escape_depth: 가진 두 칸이 이웃하지 않음");
    }
    // 가진 칸(1칸 또는 이웃 2칸)의 합집합은 축에 나란한 사각형 하나
    double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin, ymin = xmin, ymax = -xmin;
    for (const auto& c : held) {
        for (const auto& v : cell_box(c, height, cell)) {
            xmin = std::min(xmin, v.x);
            xmax = std::max(xmax, v.x);
            ymin = std::min(ymin, v.y);
            ymax = std::max(ymax, v.y);
        }
    }
    // 몸도 볼록, 영역도 볼록 사각형 → 몸이 안에 있으려면 네 꼭짓점이 다 안에 있으면 된다. 가장 많이 나간 꼭짓점의 거리.
    double depth = 0.0;
    for (const auto& v : footprint_corners(pose, fp)) {
        const double ox = std::max({xmin - v.x, v.x - xmax, 0.0});
        const double oy = std::max({ymin - v.y, v.y - ymax, 0.0});
        depth = std::max(depth, std::hypot(ox, oy));
    }
    return depth;
}

}  // namespace mapf_exec
