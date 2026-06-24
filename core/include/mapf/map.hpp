// ─────────────────────────────────────────────────────────────────
// core/include/mapf/map.hpp
//
// 02_map_and_agent.md 2.1~2.3절을 코드로 옮긴 파일.
// 격자 위의 칸(Cell), 시공간 상의 칸(SpaceTimeCell), 격자 전체(Map)를 정의한다.
// ─────────────────────────────────────────────────────────────────
#pragma once

#include <string>
#include <vector>

namespace mapf {

// 격자 위의 한 칸. x = 열(가로), y = 행(세로). 왼쪽 위가 (0,0).
struct Cell {
    int x{0};
    int y{0};

    // std::map/std::set에 Cell을 키로 쓰거나, 두 칸이 같은지 비교할 때 필요.
    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
    bool operator<(const Cell& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

// 칸 + 시각(타임스텝). Space-time A*가 탐색하는 상태의 단위.
struct SpaceTimeCell {
    int x{0};
    int y{0};
    int t{0};

    bool operator==(const SpaceTimeCell& other) const {
        return x == other.x && y == other.y && t == other.t;
    }
    bool operator<(const SpaceTimeCell& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return t < other.t;
    }
};

// 격자 전체. "가로x세로 크기"와 "어느 칸이 벽인지"를 담는다.
// 동적 장애물(다른 로봇 등 시간에 따라 막히는 것)은 다루지 않는다 — 그건
// ReservationTable(03장)의 책임이다.
class Map {
public:
    // 빈 맵(벽 없음) 생성.
    Map(int width, int height);

    // ASCII 그림으로 맵 생성. '#' = 벽, 그 외 문자 = 빈 칸.
    // rows[y][x]가 (x, y) 칸에 대응한다.
    explicit Map(const std::vector<std::string>& rows);

    int width() const { return width_; }
    int height() const { return height_; }

    // 그 칸이 지나갈 수 있는 칸인가? (격자 범위 밖이거나 벽이면 false)
    bool is_passable(int x, int y) const;

    // c에서 한 스텝에 갈 수 있는 칸들: 상/하/좌/우 + 대기(자기 자신).
    // "대기"를 포함하는 이유는 02장 2.3절 참고 — 좁은 통로에서 다른 로봇을
    // 기다려야 하는 상황을 표현하려면 반드시 필요하다.
    std::vector<Cell> neighbors(Cell c) const;

    // 런타임에 정적 장애물(영원히 막힌 벽)을 추가한다.
    void set_obstacle(int x, int y);

private:
    bool in_bounds(int x, int y) const;

    int width_;
    int height_;
    std::vector<std::vector<bool>> blocked_;  // blocked_[y][x] == true면 벽
};

}  // namespace mapf
