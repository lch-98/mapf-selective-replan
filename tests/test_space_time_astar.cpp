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

// ───────────────────────────────────────────────────────────────
// 17장 결정 7 — 작업 시간(service_time)
// ───────────────────────────────────────────────────────────────

TEST(SpaceTimeAStarTest, ServiceTimeAppendsStayCellsAtGoal) {
    // 5x1 통로. (0,0) → (2,0)은 t=2에 도착한다. 작업 시간 3이면 t=2~4에 작업하고
    // t=5부터 움직일 수 있으므로, 경로 끝에 (2,0)t3 (2,0)t4 (2,0)t5가 붙는다.
    Map map(5, 1);
    ReservationTable reservations;
    SpaceTimeAStar astar(map, reservations);

    auto path = astar.search(Cell{0, 0}, Cell{2, 0}, /*start_time=*/0, /*service_time=*/3);

    ASSERT_TRUE(path.has_value());
    ASSERT_EQ(path->size(), 6u);
    EXPECT_EQ((*path)[2], (SpaceTimeCell{2, 0, 2}));  // 도착
    EXPECT_EQ((*path)[3], (SpaceTimeCell{2, 0, 3}));
    EXPECT_EQ((*path)[4], (SpaceTimeCell{2, 0, 4}));
    EXPECT_EQ((*path)[5], (SpaceTimeCell{2, 0, 5}));  // 작업 끝, 이 시각부터 움직일 수 있음
}

TEST(SpaceTimeAStarTest, ServiceTimeWhenAlreadyAtGoal) {
    // 출발 칸이 곧 목적지면 출발 시각에 도착한 것이다. 작업 시간 2 → t=0~1 작업, t=2부터 이동 가능.
    Map map(3, 1);
    ReservationTable reservations;
    SpaceTimeAStar astar(map, reservations);

    auto path = astar.search(Cell{1, 0}, Cell{1, 0}, /*start_time=*/0, /*service_time=*/2);

    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(*path, (Path{SpaceTimeCell{1, 0, 0}, SpaceTimeCell{1, 0, 1}, SpaceTimeCell{1, 0, 2}}));
}

TEST(SpaceTimeAStarTest, ServiceTimeMustFinishWithinHorizon) {
    // (0,0) → (2,0)은 가장 빨리 t=2에 도착한다. max_timestep=4.
    //   작업 2: 2 + 2 = 4 <= 4 → 성공, 경로는 t=4에서 끝난다.
    //   작업 3: 2 + 3 = 5 >  4 → 시간 안에 작업이 안 끝나므로 실패.
    Map map(3, 1);
    ReservationTable reservations;
    AStarConfig config;
    config.max_timestep = 4;
    SpaceTimeAStar astar(map, reservations, config);

    auto fits = astar.search(Cell{0, 0}, Cell{2, 0}, /*start_time=*/0, /*service_time=*/2);
    ASSERT_TRUE(fits.has_value());
    EXPECT_EQ(fits->back(), (SpaceTimeCell{2, 0, 4}));

    EXPECT_FALSE(astar.search(Cell{0, 0}, Cell{2, 0}, /*start_time=*/0, /*service_time=*/3).has_value());
}

TEST(SpaceTimeAStarTest, ServiceTimeFailsAtOnceWhenGoalFreesTooLate) {
    // 다른 로봇(7번)이 t=3에 목적지 (2,0)을 지나간다. 그래서 t=4 이후에야 도착할 수 있다.
    // max_timestep=5.
    //   작업 1: 4 + 1 = 5 <= 5 → 성공.
    //   작업 3: 4 + 3 = 7 >  5 → 탐색해 볼 필요도 없이 바로 실패해야 하고, 누가 막았는지
    //           추적할 수 있게 목적지 칸 (2,0)t3 하나만 blocked_attempts에 남아야 한다.
    Map map(3, 1);
    ReservationTable reservations;
    reservations.reserve(2, 0, /*t=*/3, /*agent_id=*/7);
    AStarConfig config;
    config.max_timestep = 5;
    SpaceTimeAStar astar(map, reservations, config);

    auto short_task = astar.search(Cell{0, 0}, Cell{2, 0}, /*start_time=*/0, /*service_time=*/1);
    ASSERT_TRUE(short_task.has_value());
    EXPECT_EQ(short_task->back(), (SpaceTimeCell{2, 0, 5}));

    AStarResult long_task =
        astar.search_with_diagnostics(Cell{0, 0}, Cell{2, 0}, /*start_time=*/0, /*service_time=*/3);
    EXPECT_FALSE(long_task.path.has_value());
    EXPECT_EQ(long_task.blocked_attempts, (std::vector<SpaceTimeCell>{SpaceTimeCell{2, 0, 3}}));
}

// ───────────────────────────────────────────────────────────────
// 18장 2-1단계 — 동점 처리 규칙(f → g가 큰 쪽 → 먼저 넣은 쪽)과 expanded
// ───────────────────────────────────────────────────────────────

TEST(SpaceTimeAStarTest, TieBreakGoesStraightAndIsFixed) {
    // 빈 3x3 맵에서 (0,0) → (2,2)는 길이 4인 최단 경로가 여러 개다(f가 같은 노드가 많다).
    // 규칙대로면:
    //   - f가 같으면 g가 큰 쪽(목적지에 더 가까운 쪽)을 먼저 → 기다리기·되돌아가기 없이 곧장 간다
    //   - g도 같으면 먼저 넣은 쪽 → 이웃을 "제자리, 오른쪽, 왼쪽, 아래, 위" 순서로 넣으므로 오른쪽이 먼저
    // 그래서 항상 (0,0) (1,0) (2,0) (2,1) (2,2)가 나오고, 확인하는 상태는 이 5개뿐이다.
    Map map(3, 3);
    ReservationTable reservations;
    SpaceTimeAStar astar(map, reservations);

    AStarResult result = astar.search_with_diagnostics(Cell{0, 0}, Cell{2, 2});

    ASSERT_TRUE(result.path.has_value());
    EXPECT_EQ(*result.path, (Path{SpaceTimeCell{0, 0, 0}, SpaceTimeCell{1, 0, 1},
                                  SpaceTimeCell{2, 0, 2}, SpaceTimeCell{2, 1, 3},
                                  SpaceTimeCell{2, 2, 4}}));
    EXPECT_EQ(result.expanded, 5);

    // 같은 입력이면 몇 번을 돌려도 같은 경로다.
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(astar.search(Cell{0, 0}, Cell{2, 2}), result.path);
    }
}

TEST(SpaceTimeAStarTest, ExpandedCountsEveryStateCheckedBeforeFailing) {
    // 1x3 통로에서 목적지 (2,0)이 t1부터 끝까지 막혀 있다(다른 로봇이 이미 도착해 머묾).
    // 목적지가 끝까지 막혀 있으면 탐색 없이 바로 실패하므로 확인한 상태는 0개다.
    Map map(3, 1);
    ReservationTable reservations;
    AStarConfig config;
    config.max_timestep = 10;
    for (int t = 1; t <= config.max_timestep; ++t) reservations.reserve(2, 0, t, /*agent_id=*/7);
    SpaceTimeAStar astar(map, reservations, config);

    AStarResult blocked_goal = astar.search_with_diagnostics(Cell{0, 0}, Cell{2, 0});
    EXPECT_FALSE(blocked_goal.path.has_value());
    EXPECT_EQ(blocked_goal.expanded, 0);

    // 목적지는 비어 있지만 가는 길 (1,0)이 끝까지 막혀 있으면, 출발 칸 (0,0)에서 t0~t10을
    // 전부 확인한 뒤에야 실패한다 → 11개.
    ReservationTable wall_robot;
    for (int t = 0; t <= config.max_timestep; ++t) wall_robot.reserve(1, 0, t, /*agent_id=*/7);
    SpaceTimeAStar walled(map, wall_robot, config);
    AStarResult stuck = walled.search_with_diagnostics(Cell{0, 0}, Cell{2, 0});
    EXPECT_FALSE(stuck.path.has_value());
    EXPECT_EQ(stuck.expanded, 11);
}
