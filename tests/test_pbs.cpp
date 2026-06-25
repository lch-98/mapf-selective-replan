// ─────────────────────────────────────────────────────────────────
// tests/test_pbs.cpp
//
// 05_pbs.md에서 설명한 PBS::plan의 동작을 확인한다.
// ─────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>
#include "mapf/pbs.hpp"

using namespace mapf;

TEST(PBSTest, TwoAgentsWithoutConflictBothGetShortestPaths) {
    // 5x1 통로. 로봇0: (0,0)->(1,0). 로봇1: (4,0)->(3,0). 서로 안 겹친다.
    Map map(5, 1);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{1, 0}},
        Agent{1, Cell{4, 0}, Cell{3, 0}},
    };

    auto result = pbs.plan(agents);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ((*result)[0].size(), 2u);  // (0,0)->(1,0): t=0,1
    EXPECT_EQ((*result)[1].size(), 2u);  // (4,0)->(3,0): t=0,1
}

TEST(PBSTest, LowerPriorityAgentDetoursAroundHigherPriority) {
    // 3x3 빈 격자. 가운데 행(y=1)이 (0,1)-(1,1)-(2,1)로 통로처럼 뚫려 있지만,
    // 위(y=0)/아래(y=2) 행으로 돌아갈 공간도 있다 — 즉 "비켜설 자리"가 있는 맵.
    //
    // 로봇0(1순위): (0,1)->(2,1) — 가운데 행을 직선으로 가로지른다.
    // 로봇1(2순위): (2,1)->(0,1) — 정면으로 마주치는 경로지만, 위/아래로
    //   돌아갈 공간이 있으므로 PBS가 충돌 없이 둘 다 성공시켜야 한다.
    Map map(3, 3);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 1}, Cell{2, 1}},
        Agent{1, Cell{2, 1}, Cell{0, 1}},
    };

    auto result = pbs.plan(agents);

    ASSERT_TRUE(result.has_value());
    // 로봇0은 자유롭게 직선 최단 경로(2턴)를 그대로 가져간다.
    const Path& path0 = (*result)[0];
    EXPECT_EQ(path0.size(), 3u);  // t=0,1,2
    EXPECT_EQ(path0.front(), (SpaceTimeCell{0, 1, 0}));
    EXPECT_EQ(path0.back(), (SpaceTimeCell{2, 1, 2}));

    // 로봇1은 로봇0이 가운데 행을 차지하는 시간대를 피해 위/아래로 돌아가야
    // 하므로, 직선 2턴보다 더 걸린다.
    const Path& path1 = (*result)[1];
    EXPECT_GT(path1.size(), 3u);
    EXPECT_EQ(path1.front(), (SpaceTimeCell{2, 1, 0}));
    EXPECT_EQ(path1.back().x, 0);
    EXPECT_EQ(path1.back().y, 1);
}

TEST(PBSTest, FailsWhenAnyAgentIsUnreachable) {
    Map map({
        "...",
        ".#.",
        "...",
    });
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{1, 1}},  // (1,1)은 사방이 벽 — 도달 불가능
    };

    auto result = pbs.plan(agents);

    EXPECT_FALSE(result.has_value());
}

TEST(PBSTest, TailReservationProtectsArrivedAgentForever) {
    // 2x1 통로: (0,0)-(1,0).
    // 로봇0(1순위): (1,0)->(1,0) — 제자리. t=0부터 (1,0)을 영원히 점유(Tail).
    // 로봇1(2순위): (0,0)->(1,0) — 로봇0의 목적지로 들어가려 하므로 막혀야 한다
    //   (Tail Reservation이 없으면 로봇0이 떠난 것처럼 보여서 들어갈 수 있다).
    Map map(2, 1);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{1, 0}, Cell{1, 0}},
        Agent{1, Cell{0, 0}, Cell{1, 0}},
    };

    auto result = pbs.plan(agents);

    // 로봇1은 (1,0)에 영원히 못 들어가므로 전체 계획이 실패해야 한다.
    EXPECT_FALSE(result.has_value());
}

TEST(PBSTest, GoalPreReservationBlocksPassThroughBeforeItsTurn) {
    // 3x1 통로: (0,0)-(1,0)-(2,0).
    // 로봇0(1순위): (0,0)->(2,0) — 통로 전체를 지나가야 한다.
    // 로봇1(2순위): (1,0)->(1,0) — 제자리(목적지가 통로 중간).
    //   로봇1의 차례가 아직 안 왔어도, 목적지 임시 선점 때문에 로봇0은
    //   (1,0)을 "그냥 지나가는 길"로 쓸 수 없고 돌아갈 길이 없으므로 실패해야 한다.
    Map map(3, 1);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{2, 0}},
        Agent{1, Cell{1, 0}, Cell{1, 0}},
    };

    auto result = pbs.plan(agents);

    EXPECT_FALSE(result.has_value());
}
