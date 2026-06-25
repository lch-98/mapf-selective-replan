// ─────────────────────────────────────────────────────────────────
// tests/test_space_time_astar.cpp
//
// 04_space_time_astar.md에서 설명한 SpaceTimeAStar의 동작을 확인한다.
// ─────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>
#include "mapf/space_time_astar.hpp"

using namespace mapf;

TEST(SpaceTimeAStarTest, FindsStraightLinePathOnEmptyMap) {
    Map map(5, 1);
    ReservationTable reservations;
    SpaceTimeAStar astar(map, reservations);

    auto path = astar.search(Cell{0, 0}, Cell{4, 0});

    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (SpaceTimeCell{0, 0, 0}));
    EXPECT_EQ(path->back(), (SpaceTimeCell{4, 0, 4}));
    EXPECT_EQ(path->size(), 5u);  // t=0,1,2,3,4
}

TEST(SpaceTimeAStarTest, StartEqualsGoalReturnsSingleCellPath) {
    Map map(3, 3);
    ReservationTable reservations;
    SpaceTimeAStar astar(map, reservations);

    auto path = astar.search(Cell{1, 1}, Cell{1, 1});

    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 1u);
    EXPECT_EQ(path->front(), (SpaceTimeCell{1, 1, 0}));
}

TEST(SpaceTimeAStarTest, ReturnsNulloptWhenGoalUnreachable) {
    // goal이 사방이 벽으로 막힌 칸 — 절대 도달 불가능.
    Map map({
        "...",
        ".#.",
        "...",
    });
    ReservationTable reservations;
    SpaceTimeAStar astar(map, reservations);

    auto path = astar.search(Cell{0, 0}, Cell{1, 1});

    EXPECT_FALSE(path.has_value());
}

TEST(SpaceTimeAStarTest, WaitsForOccupiedCellToClear) {
    // 1x3 통로: (0,0)-(1,0)-(2,0). 다른 로봇이 t=1에 (1,0)을 점유하고 있으면,
    // 이 로봇은 t=1에 곧바로 들어갈 수 없으므로 (0,0)에서 한 턴 대기해야 한다.
    Map map(3, 1);
    ReservationTable reservations;
    reservations.reserve(1, 0, /*t=*/1, /*agent_id=*/9);
    SpaceTimeAStar astar(map, reservations);

    auto path = astar.search(Cell{0, 0}, Cell{2, 0});

    ASSERT_TRUE(path.has_value());
    // (0,0)에서 1턴 대기한 뒤 이동했는지 확인: t=0과 t=1에 모두 (0,0)이 있어야 한다.
    EXPECT_EQ((*path)[0], (SpaceTimeCell{0, 0, 0}));
    EXPECT_EQ((*path)[1], (SpaceTimeCell{0, 0, 1}));
    EXPECT_EQ(path->back(), (SpaceTimeCell{2, 0, 3}));
}

TEST(SpaceTimeAStarTest, AvoidsEdgeConflict) {
    // (0,0)->(1,0) 이동이 t=0에서 막혀 있으면, 그 이동을 쓸 수 없다.
    Map map(2, 1);
    ReservationTable reservations;
    reservations.reserve_edge(0, 0, 1, 0, /*t=*/0);
    SpaceTimeAStar astar(map, reservations);

    auto path = astar.search(Cell{0, 0}, Cell{1, 0});

    ASSERT_TRUE(path.has_value());
    // t=0에 바로 이동할 수 없으므로, 최소 1턴 대기 후 이동해야 한다.
    EXPECT_GE(path->size(), 3u);
    EXPECT_EQ((*path)[0], (SpaceTimeCell{0, 0, 0}));
    EXPECT_EQ((*path)[1], (SpaceTimeCell{0, 0, 1}));
}

TEST(SpaceTimeAStarTest, RespectsStartTime) {
    Map map(3, 1);
    ReservationTable reservations;
    SpaceTimeAStar astar(map, reservations);

    auto path = astar.search(Cell{0, 0}, Cell{2, 0}, /*start_time=*/5);

    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (SpaceTimeCell{0, 0, 5}));
    EXPECT_EQ(path->back(), (SpaceTimeCell{2, 0, 7}));
}

TEST(SpaceTimeAStarTest, DiagnosticsRecordBlockedAttempts) {
    Map map(3, 1);
    ReservationTable reservations;
    reservations.reserve(1, 0, /*t=*/1, /*agent_id=*/9);
    SpaceTimeAStar astar(map, reservations);

    AStarResult result = astar.search_with_diagnostics(Cell{0, 0}, Cell{2, 0});

    ASSERT_TRUE(result.path.has_value());
    // (1,0,1)으로의 이동이 막혀서 blocked_attempts에 기록되어야 한다.
    bool found = false;
    for (const auto& cell : result.blocked_attempts) {
        if (cell == (SpaceTimeCell{1, 0, 1})) found = true;
    }
    EXPECT_TRUE(found);
}
