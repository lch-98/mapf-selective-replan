// ─────────────────────────────────────────────────────────────────
// tests/test_pbs.cpp
//
// 05_pbs.md에서 설명한 PBS::plan의 동작을 확인한다.
// ─────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>
#include "mapf/pbs.hpp"

using namespace mapf;

TEST(PBSTest, RejectsAgentWhoseStartCellIsAlreadyOwnedAtThatTime) {
    // 3x1 통로: (0,0)-(1,0)-(2,0).
    // 로봇0(1순위): (1,0)->(0,0). t=0에 (1,0)에서 출발해 t=1에 (0,0)으로
    //   이동한다. 목적지가 (0,0)이므로 Tail Reservation도 (0,0)에만 걸리고
    //   (1,0)은 t=0 이후 전혀 보호받지 않는다.
    // 로봇1(2순위): (1,0)->(1,0) — 제자리. start==goal이라 A*는 시작 칸의
    //   점유 여부를 검사하지 않고 즉시 "t=0에 도착"으로 판정한다([04장]
    //   search_with_diagnostics는 다음 칸 이동시에만 점유를 확인한다).
    //
    // 로봇0과 로봇1은 실제로 t=0에 동시에 (1,0)에 있다 — 명백한 vertex
    // conflict. register_path의 vertex 등록 루프가 reserve_if_unowned를
    // 쓰지 않고 무조건 덮어쓰는 reserve()를 썼다면, 이 충돌이 조용히
    // 사라지고 plan()이 두 경로를 모두 "성공"으로 반환해버린다 — 그래서
    // 안 된다는 것을 이 테스트가 고정한다.
    Map map(3, 1);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{1, 0}, Cell{0, 0}},
        Agent{1, Cell{1, 0}, Cell{1, 0}},
    };

    auto result = pbs.plan(agents);

    EXPECT_FALSE(result.has_value());
}

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

TEST(PBSTest, FailsWhenHigherPriorityAgentForeverBlocksLowerPriorityGoal) {
    // 3x1 통로: (0,0)-(1,0)-(2,0).
    // 로봇0(1순위): (0,0)->(2,0) — (1,0)을 t=1에 "지나간다"(통로를 끝까지 가야 함).
    // 로봇1(2순위): (1,0)->(1,0) — 제자리. 로봇1의 경로는 "t=0,1,2,...,max까지
    //   영원히 (1,0)에 있다"는 뜻인데, 그중 t=1은 이미 로봇0이 차지하고 있다.
    //   즉 로봇1이 "영원히 거기 머문다"는 전제 자체가 거짓이 되는 진짜 충돌이다
    //   — 로봇1은 t=1 순간 물리적으로 어딘가 비켜야 하는데, 그런 경로는
    //   계획된 적이 없으므로 전체가 실패해야 한다.
    Map map(3, 1);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{2, 0}},
        Agent{1, Cell{1, 0}, Cell{1, 0}},
    };

    auto result = pbs.plan(agents);

    EXPECT_FALSE(result.has_value());
}

TEST(PBSTest, TailReservationRejectionIsDetectedNotSilentlyIgnored) {
    // 2x3 격자: (0,*) 세로 통로 + (1,*) 옆 칸.
    // 로봇0(1순위): (0,0)->(0,2) — (0,1)을 t=1에 지나간다.
    // 로봇1(2순위): (0,1)->(0,1) — 제자리. 로봇0과 같은 이유로, 로봇1의
    //   "영원히 머문다"는 Tail Reservation이 t=1에서 거절되어야 하고,
    //   그 거절은 register_path가 false를 반환해 plan() 전체를 실패시켜야
    //   한다 — 조용히 무시되어 "성공"으로 잘못 보고되면 안 된다.
    Map map(2, 3);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{0, 2}},
        Agent{1, Cell{0, 1}, Cell{0, 1}},
        Agent{2, Cell{1, 1}, Cell{1, 0}},
    };

    auto result = pbs.plan(agents);

    EXPECT_FALSE(result.has_value());
}

TEST(PBSTest, LowerPriorityTailReservationDoesNotErasePriorAgentVertex) {
    // 이번엔 로봇1의 목적지를 로봇0의 경로가 전혀 지나가지 않는 칸으로 두어,
    // Tail Reservation이 거절 없이 깨끗하게 등록되는 정상 케이스를 확인한다.
    // 2x3 격자: (0,*) 세로 통로 + (1,*) 옆 칸.
    // 로봇0(1순위): (0,0)->(0,2) — (0,*) 칸만 쓴다.
    // 로봇1(2순위): (1,1)->(1,1) — 옆 칸에서 제자리. 로봇0과 칸 자체가 다르므로
    //   Tail Reservation이 전혀 거절되지 않고, 로봇0의 경로도 그대로 유지된다.
    Map map(2, 3);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{0, 2}},
        Agent{1, Cell{1, 1}, Cell{1, 1}},
    };

    auto result = pbs.plan(agents);

    ASSERT_TRUE(result.has_value());
    // 로봇0의 경로는 그대로 유지되어야 한다 — 로봇1의 Tail에 침범당하지 않음.
    EXPECT_EQ((*result)[0].back(), (SpaceTimeCell{0, 2, 2}));
    for (const auto& cell : (*result)[0]) {
        for (const auto& other : (*result)[1]) EXPECT_FALSE(cell == other);
    }
}
