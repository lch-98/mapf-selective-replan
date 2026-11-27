#include "mapf_exec/lidar_detect.hpp"

#include <algorithm>
#include <cmath>

namespace mapf_exec {

bool near_known_surface(Vec2 q, const std::vector<std::string>& rows, double cell, double clearance) {
    const int h = static_cast<int>(rows.size());
    const int w = h > 0 ? static_cast<int>(rows[0].size()) : 0;
    // 테두리 벽(격자 바로 바깥): 격자 밖이거나 경계에서 clearance 안
    if (q.x < clearance || q.y < clearance || q.x > w * cell - clearance || q.y > h * cell - clearance) return true;
    const GridCell c = world_to_cell(q, h, cell);
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const GridCell g{c.x + dx, c.y + dy};
            if (g.x < 0 || g.y < 0 || g.x >= w || g.y >= h || rows[g.y][g.x] != '#') continue;
            const Vec2 m = cell_center(g, h, cell);  // 칸 사각형까지 거리(안이면 0)
            const double ex = std::max(0.0, std::abs(q.x - m.x) - cell / 2), ey = std::max(0.0, std::abs(q.y - m.y) - cell / 2);
            if (std::hypot(ex, ey) < clearance) return true;
        }
    }
    return false;
}

std::map<std::pair<int, int>, int> unknown_cells(const Pose2& pose, const ScanData& scan, const std::vector<Vec2>& other_robots,
                                                 const std::vector<std::string>& rows, double cell, const LidarParams& p) {
    const int h = static_cast<int>(rows.size());
    const int w = h > 0 ? static_cast<int>(rows[0].size()) : 0;
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    const double ox = pose.x + c * p.sensor_offset.x - s * p.sensor_offset.y;
    const double oy = pose.y + s * p.sensor_offset.x + c * p.sensor_offset.y;
    std::map<std::pair<int, int>, int> count;
    for (std::size_t k = 0; k < scan.ranges.size(); ++k) {
        const double r = scan.ranges[k];
        if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max || r > p.d_detect) continue;
        const double a = pose.yaw + scan.angle_min + static_cast<double>(k) * scan.angle_increment;
        const Vec2 hit{ox + r * std::cos(a), oy + r * std::sin(a)};
        bool near_robot = false;
        for (const auto& q : other_robots) near_robot = near_robot || std::hypot(hit.x - q.x, hit.y - q.y) < p.robot_filter_r;
        if (near_robot) continue;
        if (near_known_surface(hit, rows, cell, p.map_clearance)) continue;
        const Vec2 in{hit.x + p.push * std::cos(a), hit.y + p.push * std::sin(a)};
        const GridCell g = world_to_cell(in, h, cell);
        if (g.x < 0 || g.y < 0 || g.x >= w || g.y >= h || rows[g.y][g.x] == '#') continue;  // 격자 밖(테두리 벽)·선반
        count[{g.x, g.y}]++;
    }
    for (auto it = count.begin(); it != count.end();) it = it->second < p.min_points ? count.erase(it) : std::next(it);
    return count;
}

int ObstacleConfirmer::window_bins() const { return static_cast<int>(std::lround(t_wait_ / bin_)) + 1; }

void ObstacleConfirmer::observe(double t, const std::vector<GridCell>& seen, const std::string& by) {
    const int n = window_bins();
    for (const auto& g : seen) {
        auto [it, fresh] = cand_.try_emplace({g.x, g.y});
        Cand& c = it->second;
        if (fresh) {
            c.first = t;
            c.by = by;
        }
        c.last_seen = std::max(c.last_seen, t);
        const long idx = std::lround((t - c.first) / bin_);
        if (idx < 0) continue;
        if (std::find(c.bins.begin(), c.bins.end(), idx) == c.bins.end()) {
            c.bins.insert(std::upper_bound(c.bins.begin(), c.bins.end(), idx), idx);
        }
        // 이 순간까지의 최근 창 [idx − n + 1, idx] (처음 본 뒤 t_wait가 지나야 창이 꽉 참)
        const long lo = idx - n + 1;
        if (lo < 0) continue;
        const int hits = static_cast<int>(std::count_if(c.bins.begin(), c.bins.end(), [&](long b) { return b >= lo && b <= idx; }));
        c.seen_in_window = hits;
        c.window = n;
        if (hits >= ratio_ * n - 1e-9) c.ready = true;
    }
}

std::vector<ObstacleConfirmer::Event> ObstacleConfirmer::update(double t) {
    std::vector<Event> out;
    for (auto it = cand_.begin(); it != cand_.end();) {
        Cand& c = it->second;
        const GridCell g{it->first.first, it->first.second};
        if (!c.announced) {
            out.push_back({Event::Kind::kDetected, g, c.first, c.first, c.by, 0, 0});
            c.announced = true;
        }
        if (c.ready && !hold_confirm_) {
            out.push_back({Event::Kind::kConfirmed, g, t, c.first, c.by, c.seen_in_window, c.window});
            it = cand_.erase(it);
            continue;
        }
        if (!c.ready && t - c.last_seen > drop_after_ + 1e-9) {
            out.push_back({Event::Kind::kDropped, g, t, c.first, c.by, c.seen_in_window, c.window});
            it = cand_.erase(it);
            continue;
        }
        ++it;
    }
    return out;
}

}  // namespace mapf_exec
