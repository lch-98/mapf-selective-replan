// ─────────────────────────────────────────────────────────────────
// core/src/map.cpp
//
// map.hpp에서 선언한 Map의 실제 동작을 구현한다.
// ─────────────────────────────────────────────────────────────────
#include "mapf/map.hpp"

namespace mapf {

Map::Map(int width, int height)
    : width_(width),
      height_(height),
      blocked_(height, std::vector<bool>(width, false)) {}

Map::Map(const std::vector<std::string>& rows)
    : width_(rows.empty() ? 0 : static_cast<int>(rows[0].size())),
      height_(static_cast<int>(rows.size())),
      blocked_(height_, std::vector<bool>(width_, false)) {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            blocked_[y][x] = (rows[y][x] == '#');
        }
    }
}

bool Map::in_bounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool Map::is_passable(int x, int y) const {
    if (!in_bounds(x, y)) return false;
    return !blocked_[y][x];
}

std::vector<Cell> Map::neighbors(Cell c) const {
    std::vector<Cell> result;

    // 후보: 상/하/좌/우 + 대기(자기 자신). 대기를 포함하는 이유는
    // map.hpp의 주석(및 02장 2.3절) 참고.
    const Cell candidates[] = {
        {c.x, c.y},      // 대기
        {c.x + 1, c.y},  // 오른쪽
        {c.x - 1, c.y},  // 왼쪽
        {c.x, c.y + 1},  // 아래
        {c.x, c.y - 1},  // 위
    };

    for (const Cell& cand : candidates) {
        if (is_passable(cand.x, cand.y)) {
            result.push_back(cand);
        }
    }
    return result;
}

void Map::set_obstacle(int x, int y) {
    if (in_bounds(x, y)) {
        blocked_[y][x] = true;
    }
}

}  // namespace mapf
