// ─────────────────────────────────────────────────────────────────
// core/src/reservation_table.cpp
//
// reservation_table.hpp에서 선언한 ReservationTable의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/reservation_table.hpp"

namespace mapf {

void ReservationTable::reserve(int x, int y, int t, int agent_id) {
    vertices_[VertexKey{x, y, t}] = agent_id;
}

void ReservationTable::unreserve(int x, int y, int t) {
    vertices_.erase(VertexKey{x, y, t});
}

bool ReservationTable::is_occupied(int x, int y, int t) const {
    return vertices_.find(VertexKey{x, y, t}) != vertices_.end();
}

std::optional<int> ReservationTable::get_owner(int x, int y, int t) const {
    auto it = vertices_.find(VertexKey{x, y, t});
    if (it == vertices_.end()) return std::nullopt;
    return it->second;
}

void ReservationTable::reserve_edge(int x1, int y1, int x2, int y2, int t) {
    edges_.insert(EdgeKey{x1, y1, x2, y2, t});
    edges_.insert(EdgeKey{x2, y2, x1, y1, t});  // 역방향도 같이 막는다.
}

bool ReservationTable::is_edge_occupied(int x1, int y1, int x2, int y2, int t) const {
    return edges_.find(EdgeKey{x1, y1, x2, y2, t}) != edges_.end();
}

void ReservationTable::clear() {
    vertices_.clear();
    edges_.clear();
}

}  // namespace mapf
