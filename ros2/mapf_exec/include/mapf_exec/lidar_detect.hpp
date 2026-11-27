// mapf_exec/lidar_detect.hpp — LiDAR로 장애물 칸 찾기 (7-5, docs/31 31.5, 31.25).
//
// 1. 스캔 하나 → "지도에 없는 물체" 칸(unknown_cells):
//    점마다 월드 좌표 = 센서 위치(회전 중심 + sensor_offset) + 거리 × 방향.
//    버리는 점: 거리 > d_detect, 범위 밖·무한대, **알고 있는 면(선반 칸·테두리 벽)에서 map_clearance(4 cm) 안**(지도에 있는 물체 + 잡음 1 cm),
//    **다른 로봇 회전 중심에서 robot_filter_r(0.2 m) 안**(몸 회전 원 0.14 m + 여유).
//    남은 점은 광선 방향으로 push(2 cm) 더 들어간 자리의 칸으로 센다(상자 표면은 칸 경계에서 5 cm 안쪽이라 2 cm 밀어도 그 칸).
//    한 칸에 점이 min_points(3) 개 이상이면 그 스캔에서 "보였다".
//    (처음에는 "5 cm 밀어 선반 안이면 버림"만 했는데, 선반 모서리 근처에 비스듬히 닿은 점은 밀면 모서리를 지나 옆 빈칸으로 빠져
//     장애물 없는 장면에서 오탐이 쏟아졌다 — 31.25.)
// 2. 확정(ObstacleConfirmer, 미끄러지는 창): 0.2초(스캔 주기) 칸으로 나눠, 보일 때마다 "그 순간까지의 최근 t_wait"
//    (칸 n = t_wait/0.2 + 1개) 중 90% 이상에서 보였으면 확정. drop_after(1초) 동안 한 번도 안 보이면 버림(다시 보이면 새 발견).
//    t_wait = 0이면 처음 보인 순간 확정.
//    (처음에는 "처음 보인 순간부터 고정된 창"이었는데, d_detect 경계에서 처음 보인 상자는 처음 1~2초 점이 들락날락해 42/50칸으로
//     버려졌고, HOLD가 풀린 로봇이 상자로 들어가 비상 정지했다 — 31.25.)
// ROS를 모르는 순수 코드라 gtest로 손 계산 예시를 시험한다(test_lidar_detect.cpp).
#pragma once

#include <map>
#include <string>
#include <vector>

#include "mapf_exec/geometry.hpp"

namespace mapf_exec {

struct ScanData {
    double angle_min{0.0}, angle_increment{0.0}, range_min{0.12}, range_max{3.5};
    std::vector<float> ranges;
};

struct LidarParams {
    double d_detect{2.0};
    double robot_filter_r{0.2};
    double push{0.02};
    double map_clearance{0.04};
    int min_points{3};
    Vec2 sensor_offset{-0.032, 0.0};  // 회전 중심 → LiDAR (로봇 좌표, TurtleBot3 Burger)
};

// 점 q가 알고 있는 면(선반 칸 사각형, 격자 테두리)에서 clearance 안인가(안에 있어도 참)
bool near_known_surface(Vec2 q, const std::vector<std::string>& rows, double cell, double clearance);

// rows: 지금 알고 있는 지도('#' = 막힘. 이미 확정된 장애물도 '#'로 넣어 줄 것). 결과: 칸 → 점 수(min_points 이상인 칸만).
std::map<std::pair<int, int>, int> unknown_cells(const Pose2& pose, const ScanData& scan, const std::vector<Vec2>& other_robots,
                                                 const std::vector<std::string>& rows, double cell, const LidarParams& p);

class ObstacleConfirmer {
public:
    explicit ObstacleConfirmer(double t_wait, double bin = 0.2, double ratio = 0.9, double drop_after = 1.0)
        : t_wait_(t_wait), bin_(bin), ratio_(ratio), drop_after_(drop_after) {}

    // 스캔 하나에서 보인 칸들(누가 봤는지는 처음 발견자 기록용)
    void observe(double t, const std::vector<GridCell>& seen, const std::string& by);

    struct Event {
        enum class Kind { kDetected, kConfirmed, kDropped };
        Kind kind;
        GridCell cell;
        double t;            // 사건 시각
        double first_seen;   // 처음 보인 시각
        std::string by;      // 처음 본 로봇
        int bins_seen{0}, bins_total{0};  // 확정: 확정한 창, 버림: 마지막으로 본 순간까지의 창
    };
    // t까지의 사건(새 발견은 observe 때 쌓였다가 여기서 나옴). 확정·버림된 칸은 후보에서 빠진다.
    std::vector<Event> update(double t);
    // 확정을 미룸(예: 다른 장애물 재계획 중) — 후보로 계속 둠. 기본은 미루지 않음.
    void set_hold_confirm(bool h) { hold_confirm_ = h; }
    bool is_candidate(GridCell c) const { return cand_.count({c.x, c.y}) > 0; }

private:
    struct Cand {
        double first{0.0}, last_seen{0.0};
        std::string by;
        std::vector<long> bins;  // 보인 칸 번호(처음 본 시각 기준, 오름차순, 중복 없음)
        bool announced{false}, ready{false};
        int seen_in_window{0}, window{0};
    };
    int window_bins() const;
    double t_wait_, bin_, ratio_, drop_after_;
    bool hold_confirm_{false};
    std::map<std::pair<int, int>, Cand> cand_;
};

}  // namespace mapf_exec
