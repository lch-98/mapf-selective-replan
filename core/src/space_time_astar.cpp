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

// 탐색 트리의 한 노드. g는 지금까지 걸린 타임스텝(턴) 수, h는 휴리스틱.
// 대기(wait)도 한 턴을 쓰므로 이동과 똑같이 g에 +1된다 — "칸 수"가 아니라
// "시간(비용)"이 g의 정확한 의미다.
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
    // 비교를 뒤집어야 한다(>를 쓰면 작은 f가 위로 올라온다) 즉, 오름차순 top()이 가장 작은 값을 가짐
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

// 생성자 생성
SpaceTimeAStar::SpaceTimeAStar(const Map& map, const ReservationTable& reservations,
                                AStarConfig config)
    : map_(map), reservations_(reservations), config_(config) {}

// search_with_diagnostics의 wrapper 함수 > 경로만 반환할 때 사용
std::optional<Path> SpaceTimeAStar::search(Cell start, Cell goal, int start_time) const {
    return search_with_diagnostics(start, goal, start_time).path;
}

AStarResult SpaceTimeAStar::search_with_diagnostics(Cell start, Cell goal,
                                                     int start_time) const {
    AStarResult result;

    // 목적지가 마지막으로 예약된 시각. PBS는 경로를 "도착 후 목적지에 영원히
    // 머문다"(Tail Reservation)고 보고 등록하므로, 이 시각보다 뒤에 도착해야만
    // 끝까지 머물 수 있다. 도착 순간에 멈추고 이걸 안 보면, 나중에 누가 목적지를
    // 지나갈 때 "기다렸다 늦게 도착하면 풀리는" 경우에도 등록 단계에서 실패한다.
    // 예약이 하나도 없으면 start_time - 1로 둔다(어느 시각에 도착해도 된다).
    int goal_last_reserved = start_time - 1;
    for (int t = start_time; t <= config_.max_timestep; ++t) {
        if (reservations_.is_occupied(goal.x, goal.y, t)) goal_last_reserved = t;
    }

    // max_timestep까지 목적지가 막혀 있으면(장애물, 또는 목적지가 같은 앞 순서
    // 로봇의 Tail) 언제 도착해도 머물 수 없다 — 탐색해봐야 max_timestep까지
    // 헛돌기만 하므로 바로 실패한다. 막은 칸은 blocked_attempts로 알려준다.
    if (goal_last_reserved >= config_.max_timestep) {
        result.blocked_attempts.push_back(SpaceTimeCell{goal.x, goal.y, goal_last_reserved});
        return result;
    }
    bool goal_block_reported = false;

    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare> open;
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
            if (current->cell.t > goal_last_reserved) {
                result.path = reconstruct_path(current);
                return result;
            }
            // 도착은 했지만 나중에 누가 목적지를 지나가서 머물 수 없다 — 도착으로
            // 치지 않고 계속 탐색한다(비켜 있다가/기다렸다가 늦게 도착하는 길).
            // 끝내 못 찾으면 원인 추적(06장 Tier 1)에 쓰도록 막은 칸을 남긴다.
            if (!goal_block_reported) {
                result.blocked_attempts.push_back(
                    SpaceTimeCell{goal.x, goal.y, goal_last_reserved});
                goal_block_reported = true;
            }
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
            // 위 visited.insert()로 최적의 g값을 가지는 칸을 선점 했으므로, 즉 최적의 칸을 저장
            // 아래 visited.count()는 이미 visited 안의 최적의 칸을 넘어간다는 뜻 (불필요한 연산을 줄이는 최적화)
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
