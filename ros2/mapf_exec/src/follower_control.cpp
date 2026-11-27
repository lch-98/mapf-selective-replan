#include "mapf_exec/follower_control.hpp"

#include <algorithm>
#include <cmath>

namespace mapf_exec {

SegmentError segment_error(const Pose2& pose, const Segment& seg) {
    const double dx = seg.goal.x - seg.start.x, dy = seg.goal.y - seg.start.y;
    const double len = std::hypot(dx, dy);
    SegmentError e;
    if (len < 1e-9) return e;
    const double ux = dx / len, uy = dy / len;
    const double rx = seg.goal.x - pose.x, ry = seg.goal.y - pose.y;      // 목표까지
    const double sx = pose.x - seg.start.x, sy = pose.y - seg.start.y;    // 출발점에서
    e.along = rx * ux + ry * uy;
    e.cross = ux * sy - uy * sx;  // 왼쪽 +
    e.heading = wrap_angle(std::atan2(uy, ux) - pose.yaw);
    return e;
}

double segment_length(const Segment& s) { return std::hypot(s.goal.x - s.start.x, s.goal.y - s.start.y); }

bool continues_straight(const Segment& cur, const Segment& next, double tol) {
    const double lc = segment_length(cur), ln = segment_length(next);
    if (lc < 1e-9 || ln < 1e-9) return false;
    if (std::hypot(next.start.x - cur.goal.x, next.start.y - cur.goal.y) > tol) return false;
    const double dot = ((cur.goal.x - cur.start.x) * (next.goal.x - next.start.x) +
                        (cur.goal.y - cur.start.y) * (next.goal.y - next.start.y)) / (lc * ln);
    return dot > std::cos(M_PI / 180.0);
}

ControlOutput follow_step(const Pose2& pose, const Segment& seg, Phase phase, const FollowerParams& p, double carry) {
    if (std::hypot(seg.goal.x - seg.start.x, seg.goal.y - seg.start.y) < 1e-9) return {0.0, 0.0, Phase::kDone};
    if (phase == Phase::kDone) return {0.0, 0.0, Phase::kDone};

    const SegmentError e = segment_error(pose, seg);
    const bool pass = carry > 0.0;
    if (e.along <= (pass ? 0.0 : p.stop_along)) return {0.0, 0.0, Phase::kDone};

    if (phase == Phase::kDrive && std::abs(e.heading) > p.rotate_again) phase = Phase::kRotate;

    if (phase == Phase::kRotate) {
        if (std::abs(e.heading) <= p.heading_tol) {
            phase = Phase::kDrive;  // 아래 Drive로 이어서 계산
        } else {
            const double w = std::clamp(p.k_heading * std::abs(e.heading), p.min_angular, p.max_angular);
            return {0.0, std::copysign(w, e.heading), Phase::kRotate};
        }
    }

    // Drive: 방향 차이 + 옆 오차를 함께 줄인다(왼쪽으로 벗어났으면 오른쪽으로 돌기 → −k_cross·cross)
    const double w = std::clamp(p.k_heading * e.heading - p.k_cross * e.cross, -p.max_angular, p.max_angular);
    const double v = std::clamp(p.k_dist * (e.along + (pass ? carry : 0.0)), p.min_linear, p.max_linear);
    return {v, w, Phase::kDrive};
}

}  // namespace mapf_exec
