// ─────────────────────────────────────────────────────────────────
// tests/test_map_and_agent.cpp
//
// 02_map_and_agent.md에서 설명한 Cell/Map/Agent의 동작을 확인한다.
// ─────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>
#include <stdexcept>
#include "mapf/map.hpp"
#include "mapf/agent.hpp"

using namespace mapf;

TEST(MapTest, EmptyMapIsAllPassable) {
    Map map(3, 3);
    EXPECT_TRUE(map.is_passable(0, 0));
    EXPECT_TRUE(map.is_passable(2, 2));
}

TEST(MapTest, AsciiMapMarksWallsAndFloors) {
    // 행(y)이 위에서부터: y=0 "#.#", y=1 "...", y=2 "#.#"
    Map map({
        "#.#",
        "...",
        "#.#",
    });
    EXPECT_FALSE(map.is_passable(0, 0));  // '#'
    EXPECT_TRUE(map.is_passable(1, 0));   // '.'
    EXPECT_FALSE(map.is_passable(2, 0));  // '#'
    EXPECT_TRUE(map.is_passable(1, 1));   // '.'
}

TEST(MapTest, OutOfBoundsIsNotPassable) {
    Map map(3, 3);
    EXPECT_FALSE(map.is_passable(-1, 0));
    EXPECT_FALSE(map.is_passable(0, -1));
    EXPECT_FALSE(map.is_passable(3, 0));
    EXPECT_FALSE(map.is_passable(0, 3));
}

TEST(MapTest, NeighborsIncludesWait) {
    Map map(3, 3);
    auto result = map.neighbors(Cell{1, 1});

    bool found_self = false;
    for (const Cell& c : result) {
        if (c == Cell{1, 1}) found_self = true;
    }
    EXPECT_TRUE(found_self);
}

TEST(MapTest, NeighborsExcludesWallsAndOutOfBounds) {
    // 중앙 칸 (1,1)이 사방이 벽으로 둘러싸인 맵.
    Map map({
        "###",
        "#.#",
        "###",
    });
    auto result = map.neighbors(Cell{1, 1});

    // 대기(자기 자신)만 가능해야 한다.
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0], (Cell{1, 1}));
}

TEST(MapTest, NeighborsOfCornerInOpenMapHasThreeOptions) {
    // 3x3 빈 맵의 모서리 (0,0): 대기 + 오른쪽 + 아래, 총 3개.
    Map map(3, 3);
    auto result = map.neighbors(Cell{0, 0});
    EXPECT_EQ(result.size(), 3u);
}

TEST(MapTest, SetObstacleBlocksThatCell) {
    Map map(3, 3);
    EXPECT_TRUE(map.is_passable(1, 1));

    map.set_obstacle(1, 1);
    EXPECT_FALSE(map.is_passable(1, 1));
}

TEST(MapTest, NegativeWidthOrHeightThrows) {
    EXPECT_THROW(Map(-1, 3), std::invalid_argument);
    EXPECT_THROW(Map(3, -1), std::invalid_argument);
}

TEST(MapTest, MismatchedRowLengthsThrows) {
    EXPECT_THROW(Map({
                      "...",
                      "..",  // 길이가 다른 행
                      "...",
                  }),
                  std::invalid_argument);
}

TEST(AgentTest, HoldsIdStartGoal) {
    Agent agent{42, Cell{0, 0}, Cell{2, 2}};
    EXPECT_EQ(agent.id, 42);
    EXPECT_EQ(agent.start, (Cell{0, 0}));
    EXPECT_EQ(agent.goal, (Cell{2, 2}));
}
