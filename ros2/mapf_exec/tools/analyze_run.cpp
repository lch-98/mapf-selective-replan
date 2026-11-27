// analyze_run — 주행 기록(CSV)을 읽어 몸 안전(31.4 (다))과 추종 오차를 계산한다. 7-2부터 모든 3D 실행 기록에 쓴다.
//
//   analyze_run --map rows.txt --cell 0.5 --log run.csv
//
// rows.txt: 시나리오 격자 한 행씩('#' = 선반). run.csv 열(첫 줄 머리말):
//   t,robot,x,y,yaw,kind,held,seg_sx,seg_sy,seg_gx,seg_gy
//   - t: 시뮬레이터 시각(초). 같은 t의 줄끼리 로봇 사이 겹침을 본다.
//   - kind: MOVE / WAIT / IDLE,  held: 그 순간 가진 칸 "열:행" 또는 "열:행;열:행"
//   - seg_*: MOVE일 때 구간(출발 칸 중심 → 목표 칸 중심, 월드 m)
// 출력(JSON, 표준 출력): 로봇마다 칸 이탈·옆 오차·지나침·선반/벽 겹침, 로봇 쌍마다 몸 겹침·최소 거리.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "mapf_exec/follower_control.hpp"
#include "mapf_exec/geometry.hpp"

using namespace mapf_exec;

namespace {

struct Row {
    double t{0};
    std::string robot, kind;
    Pose2 pose;
    std::vector<GridCell> held;
    Segment seg;
};

struct RobotStats {
    long samples{0}, escape_samples{0}, shelf_samples{0};
    double max_escape{0}, max_cross{0}, max_overshoot{0};
};

struct PairStats {
    long checked{0}, overlap_samples{0};
    double min_distance{std::numeric_limits<double>::infinity()};
};

std::vector<std::string> split(const std::string& s, char d) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, d)) out.push_back(item);
    return out;
}

int usage() {
    std::cerr << "usage: analyze_run --map rows.txt --cell 0.5 --log run.csv\n";
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    std::string map_path, log_path;
    double cell = 0.5;
    for (int i = 1; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        if (k == "--map") map_path = argv[i + 1];
        else if (k == "--log") log_path = argv[i + 1];
        else if (k == "--cell") cell = std::stod(argv[i + 1]);
        else return usage();
    }
    if (map_path.empty() || log_path.empty()) return usage();

    std::vector<std::string> rows;
    {
        std::ifstream f(map_path);
        std::string line;
        while (std::getline(f, line)) {
            if (!line.empty()) rows.push_back(line);
        }
    }
    if (rows.empty()) {
        std::cerr << "지도가 비었음: " << map_path << "\n";
        return 2;
    }
    const int h = static_cast<int>(rows.size()), w = static_cast<int>(rows[0].size());
    std::vector<Corners> shelves;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (rows[y][x] == '#') shelves.push_back(cell_box({x, y}, h, cell));
        }
    }

    std::map<std::string, RobotStats> robots;
    std::map<std::pair<std::string, std::string>, PairStats> pairs;
    std::ifstream f(log_path);
    std::string line;
    std::getline(f, line);  // 머리말
    double cur_t = std::numeric_limits<double>::quiet_NaN();
    std::vector<Row> same_t;

    auto flush_pairs = [&]() {
        for (std::size_t i = 0; i < same_t.size(); ++i) {
            for (std::size_t j = i + 1; j < same_t.size(); ++j) {
                const auto& a = same_t[i];
                const auto& b = same_t[j];
                auto key = a.robot < b.robot ? std::make_pair(a.robot, b.robot) : std::make_pair(b.robot, a.robot);
                auto& ps = pairs[key];
                Corners ca = footprint_corners(a.pose), cb = footprint_corners(b.pose);
                ps.checked++;
                if (convex_overlap(ca, cb)) ps.overlap_samples++;
                ps.min_distance = std::min(ps.min_distance, convex_distance(ca, cb));
            }
        }
        same_t.clear();
    };

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto c = split(line, ',');
        if (c.size() < 11) {
            std::cerr << "열이 모자란 줄: " << line << "\n";
            return 2;
        }
        Row r;
        r.t = std::stod(c[0]);
        r.robot = c[1];
        r.pose = {std::stod(c[2]), std::stod(c[3]), std::stod(c[4])};
        r.kind = c[5];
        for (const auto& hc : split(c[6], ';')) {
            auto xy = split(hc, ':');
            if (xy.size() == 2) r.held.push_back({std::stoi(xy[0]), std::stoi(xy[1])});
        }
        r.seg = {{std::stod(c[7]), std::stod(c[8])}, {std::stod(c[9]), std::stod(c[10])}};

        if (r.t != cur_t) {
            flush_pairs();
            cur_t = r.t;
        }
        // 같은 시각에 같은 로봇이 또 나오면 건너뜀(기록하는 쪽 타이머가 같은 시뮬레이터 시각에 두 번 돌 수 있음, 31.22).
        // 안 그러면 로봇이 자기 자신과 "겹친" 것으로 세어진다.
        bool dup = false;
        for (const auto& o : same_t) dup = dup || o.robot == r.robot;
        if (dup) continue;
        same_t.push_back(r);

        auto& st = robots[r.robot];
        st.samples++;
        if (!r.held.empty()) {
            double d = escape_depth(r.pose, r.held, h, cell);
            if (d > 0.0) st.escape_samples++;
            st.max_escape = std::max(st.max_escape, d);
        }
        Corners body = footprint_corners(r.pose);
        bool touch = false;
        for (const auto& s : shelves) touch = touch || convex_overlap(body, s);
        for (const auto& v : body) {  // 테두리 벽은 격자 바로 바깥
            touch = touch || v.x < 0.0 || v.y < 0.0 || v.x > w * cell || v.y > h * cell;
        }
        if (touch) st.shelf_samples++;
        if (r.kind == "MOVE" && std::hypot(r.seg.goal.x - r.seg.start.x, r.seg.goal.y - r.seg.start.y) > 1e-9) {
            auto e = segment_error(r.pose, r.seg);
            st.max_cross = std::max(st.max_cross, std::abs(e.cross));
            st.max_overshoot = std::max(st.max_overshoot, -e.along);
        }
    }
    flush_pairs();

    std::printf("{\n  \"robots\": {");
    bool first = true;
    for (const auto& [name, s] : robots) {
        std::printf("%s\n    \"%s\": {\"samples\": %ld, \"escape_samples\": %ld, \"max_escape_m\": %.4f, "
                    "\"max_cross_m\": %.4f, \"max_overshoot_m\": %.4f, \"shelf_or_wall_overlap_samples\": %ld}",
                    first ? "" : ",", name.c_str(), s.samples, s.escape_samples, s.max_escape, s.max_cross,
                    s.max_overshoot, s.shelf_samples);
        first = false;
    }
    std::printf("\n  },\n  \"pairs\": {");
    first = true;
    for (const auto& [key, p] : pairs) {
        std::printf("%s\n    \"%s|%s\": {\"checked\": %ld, \"overlap_samples\": %ld, \"min_distance_m\": %.4f}",
                    first ? "" : ",", key.first.c_str(), key.second.c_str(), p.checked, p.overlap_samples,
                    p.min_distance);
        first = false;
    }
    std::printf("\n  }\n}\n");
    return 0;
}
