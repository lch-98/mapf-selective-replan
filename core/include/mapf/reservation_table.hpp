// ─────────────────────────────────────────────────────────────────
// core/include/mapf/reservation_table.hpp
//
// 03_reservation_table.md를 코드로 옮긴 파일.
// "누가 언제 어디 있는지"를 기록하는 장부.
// ─────────────────────────────────────────────────────────────────
#pragma once

#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace mapf {

// (x, y, t) 세 정수를 하나의 키로 묶는다. unordered_map의 키로 쓰려면
// 해시 함수가 필요한데, 그건 아래의 VertexKeyHash가 담당한다.
struct VertexKey {
    int x;
    int y;
    int t;

    bool operator==(const VertexKey& other) const {
        return x == other.x && y == other.y && t == other.t;
    }
};

struct VertexKeyHash {
    size_t operator()(const VertexKey& k) const {
        // 세 정수를 적당히 섞어서 하나의 해시값으로 만든다.
        // (충돌이 가끔 있어도 unordered_map은 operator==로 최종 구분하므로 안전하다.)
        size_t h = std::hash<int>()(k.x);
        h = h * 31 + std::hash<int>()(k.y);
        h = h * 31 + std::hash<int>()(k.t);
        return h;
    }
};

// 두 칸 사이의 이동(t → t+1)을 키로 묶는다. Edge 예약 전용.
struct EdgeKey {
    int x1, y1, x2, y2, t;

    bool operator==(const EdgeKey& other) const {
        return x1 == other.x1 && y1 == other.y1 &&
               x2 == other.x2 && y2 == other.y2 && t == other.t;
    }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& k) const {
        size_t h = std::hash<int>()(k.x1);
        h = h * 31 + std::hash<int>()(k.y1);
        h = h * 31 + std::hash<int>()(k.x2);
        h = h * 31 + std::hash<int>()(k.y2);
        h = h * 31 + std::hash<int>()(k.t);
        return h;
    }
};

// "어느 칸이 몇 시에 누구에게 예약되어 있는지" 적은 장부.
// 정적 장애물(Map)과 달리, 여기 기록된 점유는 특정 시각(t)에만 유효하다.
class ReservationTable {
public:
    // (x,y,t)를 agent_id가 점유한다고 기록한다.
    // agent_id를 생략하면 -1("주인 모름")로 기록된다 — 점유 여부는 여전히
    // 정확히 유지되지만, 누구 것인지는 추적하지 않는다는 뜻이다.
    //
    // 주의: 이미 다른 agent_id가 점유한 칸이라도 무조건 덮어쓴다. 호출하는
    // 쪽이 "여기는 비어있거나 내 것"이라고 이미 확신할 때만 써야 한다.
    void reserve(int x, int y, int t, int agent_id = -1);

    // (x,y,t)가 비어있거나 이미 agent_id 자신의 것일 때만 예약한다.
    // 이미 다른 agent_id가 점유한 칸이면 아무 일도 하지 않고 false를
    // 반환한다 — reserve()와 달리 절대 남의 점유를 덮어쓰지 않는다.
    //
    // 왜 필요한가: PBS(05장)의 register_path가 Tail Reservation(도착
    // 시각부터 max_timestep까지 목적지를 미리 채워두는 것)을 걸 때, 그
    // 범위 안에 이미 다른(더 높은 순위) 로봇이 정당하게 등록해둔 vertex가
    // 있다면 절대 지우면 안 된다. reserve()를 그대로 쓰면 무조건 덮어써서
    // 그 로봇의 점유 기록이 사라지는 버그가 생긴다.
    bool reserve_if_unowned(int x, int y, int t, int agent_id);

    // (x,y,t)의 점유 기록을 지운다 — 누구 것이든 무조건 지운다.
    void unreserve(int x, int y, int t);

    // (x,y,t)의 주인이 정확히 agent_id일 때만 지운다. 주인이 다르거나
    // 점유가 없으면 아무 일도 하지 않는다.
    //
    // 왜 필요한가: PBS(05장)가 "내 목적지의 임시 선점만 풀겠다"고 할 때,
    // 만약 그 칸에 이미 다른 로봇의 Tail Reservation(영구 점유)이 걸려
    // 있다면 unreserve()로는 그것까지 같이 지워버린다 — 그건 잘못이다.
    // 이 함수는 "내가 등록한 것만 정확히 골라서" 지우게 해준다.
    void unreserve_if_owned_by(int x, int y, int t, int agent_id);

    // (x,y,t)가 누군가에게 점유되어 있는가?
    bool is_occupied(int x, int y, int t) const;

    // (x,y,t)를 점유한 로봇의 id. 점유되어 있지 않으면 nullopt.
    // (agent_id가 -1로 기록된 경우 -1을 반환한다 — "주인 모름"과
    //  "점유 안 됨(nullopt)"은 서로 다른 상태이므로 구분한다.)
    std::optional<int> get_owner(int x, int y, int t) const;

    // 타임스텝 t→t+1 사이의 (x1,y1)→(x2,y2) 이동을 막는다.
    // 역방향 (x2,y2)→(x1,y1)도 함께 막힌다 — 두 로봇이 마주보고
    // 스쳐 지나가는 edge conflict를 양쪽에서 잡기 위함.
    void reserve_edge(int x1, int y1, int x2, int y2, int t);

    // 타임스텝 t→t+1 사이의 (x1,y1)→(x2,y2) 이동이 막혀 있는가?
    bool is_edge_occupied(int x1, int y1, int x2, int y2, int t) const;

    // 테이블 전체를 비운다.
    void clear();

private:
    std::unordered_map<VertexKey, int, VertexKeyHash> vertices_;
    std::unordered_set<EdgeKey, EdgeKeyHash> edges_;
};

}  // namespace mapf
