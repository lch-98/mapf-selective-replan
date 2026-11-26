#include "mapf_exec/plan_check.hpp"

#include <algorithm>
#include <cstdlib>
#include <sstream>

namespace mapf_exec {

std::string Violation::describe() const {
    static const char* names[] = {"빈 경로", "격자 밖", "선반 칸", "한 번에 한 칸 넘게", "같은 칸 같은 번호", "자리 맞바꾸기",
                                  "장애물 칸"};
    std::ostringstream os;
    os << names[static_cast<int>(kind)] << ": 로봇 " << robot;
    if (other >= 0) os << " ↔ 로봇 " << other;
    if (t >= 0) os << ", 번호 " << t;
    os << ", " << cell.y << "행 " << cell.x << "열";
    return os.str();
}

namespace {

GridCell at(const std::vector<GridCell>& path, int t) {
    return path[std::min<std::size_t>(static_cast<std::size_t>(t), path.size() - 1)];
}

}  // namespace

std::vector<Violation> check_plan(const std::vector<std::string>& rows,
                                  const std::map<int, std::vector<GridCell>>& paths,
                                  const std::vector<TimedObstacle>& obstacles) {
    std::vector<Violation> out;
    const int h = static_cast<int>(rows.size());
    const int w = h > 0 ? static_cast<int>(rows[0].size()) : 0;

    int horizon = 0;
    std::vector<int> ids;
    for (const auto& [id, path] : paths) {
        if (path.empty()) {
            out.push_back({ViolationKind::kEmptyPath, id, -1, -1, {}});
            continue;
        }
        ids.push_back(id);
        horizon = std::max(horizon, static_cast<int>(path.size()));
    }

    // 로봇 하나씩: 격자 밖, 선반, 순간이동, 장애물
    for (int id : ids) {
        const auto& path = paths.at(id);
        for (int t = 0; t < static_cast<int>(path.size()); ++t) {
            const GridCell c = path[t];
            if (c.x < 0 || c.y < 0 || c.x >= w || c.y >= h) {
                out.push_back({ViolationKind::kOffMap, id, -1, t, c});
                continue;
            }
            if (rows[c.y][c.x] == '#') out.push_back({ViolationKind::kBlocked, id, -1, t, c});
            if (t > 0 && std::abs(c.x - path[t - 1].x) + std::abs(c.y - path[t - 1].y) > 1) {
                out.push_back({ViolationKind::kJump, id, -1, t, c});
            }
        }
        // 장애물: 생긴 번호부터, 경로가 끝난 뒤 머무는 칸까지
        for (const auto& ob : obstacles) {
            for (int t = ob.from_t; t < std::max(horizon, ob.from_t + 1); ++t) {
                if (at(path, t) == ob.cell) {
                    out.push_back({ViolationKind::kObstacle, id, -1, t, ob.cell});
                    break;
                }
            }
        }
    }

    // 로봇 둘씩: 같은 칸 같은 번호, 자리 맞바꾸기 (끝난 로봇은 마지막 칸에 머무름)
    for (std::size_t i = 0; i < ids.size(); ++i) {
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const auto& a = paths.at(ids[i]);
            const auto& b = paths.at(ids[j]);
            for (int t = 0; t < horizon; ++t) {
                if (at(a, t) == at(b, t)) {
                    out.push_back({ViolationKind::kVertex, ids[i], ids[j], t, at(a, t)});
                    break;  // 한 쌍에 한 번만 알림
                }
            }
            for (int t = 0; t + 1 < horizon; ++t) {
                if (at(a, t) != at(a, t + 1) && at(a, t) == at(b, t + 1) && at(a, t + 1) == at(b, t)) {
                    out.push_back({ViolationKind::kSwap, ids[i], ids[j], t, at(a, t)});
                    break;
                }
            }
        }
    }
    return out;
}

}  // namespace mapf_exec
