// ─────────────────────────────────────────────────────────────────
// core/src/space_time_astar.cpp
//
// space_time_astar.hpp에서 선언한 SpaceTimeAStar의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/space_time_astar.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <unordered_set>

namespace mapf {

namespace {

int manhattan_distance(Cell a, Cell b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// 탐색 트리의 한 노드. g는 지금까지 온 시간(=칸 수), h는 휴리스틱.
// parent로 부모 노드를 가리켜서, goal에 도착했을 때 경로를 거슬러 올라간다.
struct Node {
    SpaceTimeCell cell;
    int g;
    int h;
    std::shared_ptr<Node> parent;

    int f() const { return g + h; }
};

struct NodeCompare {
    // 우선순위 큐는 "가장 큰 것"을 꺼내므로, f가 작을수록 먼저 나오게 하려면
    // 비교를 뒤집어야 한다(>를 쓰면 작은 f가 위로 올라온다).
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->f() > b->f();
    }
};

// 우선순위 큐에서 같은 (x,y,t)를 두 번 펼치지 않기 위한 방문 기록.
struct VisitedHash {
    size_t operator()(const SpaceTimeCell& c) const {
        size_t h = std::hash<int>()(c.x);
        h = h * 31 + std::hash<int>()(c.y);
        h = h * 31 + std::hash<int>()(c.t);
        return h;
    }
};

Path reconstruct_path(const std::shared_ptr<Node>& goal_node) {
    Path path;
    for (std::shared_ptr<Node> n = goal_node; n != nullptr; n = n->parent) {
        path.push_back(n->cell);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace

SpaceTimeAStar::SpaceTimeAStar(const Map& map, const ReservationTable& reservations,
                                AStarConfig config)
    : map_(map), reservations_(reservations), config_(config) {}

std::optional<Path> SpaceTimeAStar::search(Cell start, Cell goal, int start_time) const {
    return search_with_diagnostics(start, goal, start_time).path;
}

AStarResult SpaceTimeAStar::search_with_diagnostics(Cell start, Cell goal,
                                                     int start_time) const {
    AStarResult result;

    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare>
        open;
    std::unordered_set<SpaceTimeCell, VisitedHash> visited;

    auto start_node = std::make_shared<Node>();
    start_node->cell = SpaceTimeCell{start.x, start.y, start_time};
    start_node->g = 0;
    start_node->h = manhattan_distance(start, goal);
    start_node->parent = nullptr;
    open.push(start_node);

    while (!open.empty()) {
        std::shared_ptr<Node> current = open.top();
        open.pop();

        if (visited.count(current->cell)) continue;
        visited.insert(current->cell);

        Cell current_xy{current->cell.x, current->cell.y};
        if (current_xy == goal) {
            result.path = reconstruct_path(current);
            return result;
        }

        if (current->cell.t >= config_.max_timestep) continue;

        int next_t = current->cell.t + 1;
        for (const Cell& next_xy : map_.neighbors(current_xy)) {
            SpaceTimeCell next_cell{next_xy.x, next_xy.y, next_t};

            if (reservations_.is_occupied(next_cell.x, next_cell.y, next_cell.t)) {
                result.blocked_attempts.push_back(next_cell);
                continue;
            }
            if (reservations_.is_edge_occupied(current_xy.x, current_xy.y, next_xy.x,
                                                next_xy.y, current->cell.t)) {
                result.blocked_attempts.push_back(next_cell);
                continue;
            }
            if (visited.count(next_cell)) continue;

            auto next_node = std::make_shared<Node>();
            next_node->cell = next_cell;
            next_node->g = current->g + 1;
            next_node->h = manhattan_distance(next_xy, goal);
            next_node->parent = current;
            open.push(next_node);
        }
    }

    return result;  // path는 nullopt로 남는다 — 경로 없음.
}

}  // namespace mapf
