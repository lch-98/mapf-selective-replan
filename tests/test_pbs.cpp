// ─────────────────────────────────────────────────────────────────
// tests/test_pbs.cpp
//
// 05_pbs.md(plan) + 06_selective_replan.md(replan)에서 설명한 동작을 확인한다.
// ─────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>
#include <algorithm>
#include <stdexcept>
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

TEST(PBSTest, DuplicateAgentIdsThrow) {
    Map map(3, 3);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{2, 2}},
        Agent{0, Cell{2, 0}, Cell{0, 2}},  // id 0이 중복
    };

    EXPECT_THROW(pbs.plan(agents), std::invalid_argument);
}

TEST(PBSTest, EmptyAgentsReturnsEmptySuccessfulResult) {
    Map map(3, 3);
    PBS pbs(map);

    auto result = pbs.plan({});

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->empty());
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

// ───────────────────────────────────────────────────────────────
// 06_selective_replan.md — PBS::replan
// ───────────────────────────────────────────────────────────────

TEST(PBSReplanTest, NoAffectedAgentsReturnsExistingPathsUnchanged) {
    // 5x1 통로. 로봇0의 경로는 (0,0)->(4,0)이고, 장애물은 그 경로가 한 번도
    // 지나가지 않는 칸(y=0이 아닌 곳은 없으니, 대신 같은 행이지만 로봇이
    // 이미 지나가고 한참 지난 과거 시각이 아니라 "전혀 밟지 않는 칸"이
    // 필요하다 — 1x5 통로에는 그런 칸이 없으므로, current_time을 로봇이
    // 도착하기 한참 전으로 두고 장애물을 로봇 경로 밖의 좌표에 둔다).
    Map map(5, 1);
    PBS pbs(map);

    std::vector<Agent> agents = {Agent{0, Cell{0, 0}, Cell{4, 0}}};
    auto initial = pbs.plan(agents);
    ASSERT_TRUE(initial.has_value());

    // (10,0)은 맵 밖이라 로봇 경로가 절대 닿지 않는 칸이다.
    auto result = pbs.replan(agents, *initial, {Cell{10, 0}}, /*current_time=*/0);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->escalation_tier, 0);
    EXPECT_TRUE(result->replanned_ids.empty());
    EXPECT_EQ(result->paths.at(0), initial->at(0));
}

TEST(PBSReplanTest, Tier0SucceedsWhenAffectedAgentCanDetourAlone) {
    // 3x3 빈 격자. 로봇0(혼자): (0,1)->(2,1) 직선 경로를 받았는데, t=2 이후
    // 가운데 행에 새 장애물(1,1)이 생긴다. 위/아래로 비켜갈 공간이 있고
    // 다른 로봇이 없으므로 Tier 0만으로 충분히 성공해야 한다.
    Map map(3, 3);
    PBS pbs(map);

    std::vector<Agent> agents = {Agent{0, Cell{0, 1}, Cell{2, 1}}};
    auto initial = pbs.plan(agents);
    ASSERT_TRUE(initial.has_value());
    ASSERT_EQ(initial->at(0).back(), (SpaceTimeCell{2, 1, 2}));

    auto result = pbs.replan(agents, *initial, {Cell{1, 1}}, /*current_time=*/0);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->escalation_tier, 0);
    EXPECT_EQ(result->replanned_ids, std::vector<int>{0});
    const Path& new_path = result->paths.at(0);
    EXPECT_EQ(new_path.back().x, 2);
    EXPECT_EQ(new_path.back().y, 1);
    // (1,1)을 어느 시각에도 지나가지 않아야 한다.
    for (const auto& cell : new_path) {
        EXPECT_FALSE(cell.x == 1 && cell.y == 1);
    }
}

TEST(PBSReplanTest, Tier1RescuesBlockingAgentWhenAloneDetourImpossible) {
    // 4x3 격자.
    // 로봇0(1순위, 영향받은 로봇 R): (0,0)->(3,0). current_time=0에 (1,0),(2,0)에
    //   새 장애물이 생기면 y=1행으로 우회해야 한다:
    //   (0,0)t0 (0,1)t1 (1,1)t2 (2,1)t3 (3,1)t4 (3,0)t5.
    // 로봇1(2순위, "구조 로봇" K, working_ids에 없는 상태로 시작): 장애물과 전혀
    //   안 겹치는 "옛 경로"를 손으로 구성한다 — (1,2)에서 출발해 (1,1)을 t=2에
    //   "지나가고"(목적지가 아니라 경유지) 다시 (1,2)로 돌아온다. 목적지가
    //   (1,2)라서 Tail은 (1,2)에 걸리고 R의 우회로(y=1)와 전혀 안 겹친다.
    //
    // K의 경로는 장애물과 안 겹치므로 1단계 영향 판별에서 working_ids에
    // 안 들어간다. 그런데 Tier 0에서 R의 새 경로가 등록된 뒤 K를 barrier로
    // 등록하려 하면, K가 (1,1)을 지나가는 시각(t=2)에 R도 정확히 거기 있어
    // (R의 우회 경로상 t=2의 위치) vertex 충돌로 거절된다 — 이건 06장
    // 6.5절이 말하는 "K가 비켜주면 풀리는데 못 비키는" 상황이다. Tier 1은
    // 거절당한 로봇(K) 자신을 working_ids에 추가해서 재탐색하게 하고, K는
    // 목적지(1,2)가 R의 경로 밖이므로 A*가 그 시각만 피해 자유롭게 새
    // 경로를 찾을 수 있다.
    Map map(4, 3);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{3, 0}},  // R
        Agent{1, Cell{1, 2}, Cell{1, 2}},  // K
    };

    PBSResult previous_paths;
    previous_paths[0] = Path{
        SpaceTimeCell{0, 0, 0},
        SpaceTimeCell{1, 0, 1},
        SpaceTimeCell{2, 0, 2},
        SpaceTimeCell{3, 0, 3},
    };
    previous_paths[1] = Path{
        SpaceTimeCell{1, 2, 0},
        SpaceTimeCell{1, 2, 1},
        SpaceTimeCell{1, 1, 2},
        SpaceTimeCell{1, 2, 3},
    };

    auto result = pbs.replan(agents, previous_paths, {Cell{1, 0}, Cell{2, 0}},
                              /*current_time=*/0);

    ASSERT_TRUE(result.has_value());
    // Tier 0만으로는 풀리지 않는 시나리오이므로 1단계 확장으로 풀렸어야 한다
    // (안전망 폴백인 -1이 아니라 진짜 Tier 1로 풀렸어야 한다).
    EXPECT_EQ(result->escalation_tier, 1);
    // K(로봇1)가 "구조 로봇"으로 working_ids에 추가됐어야 한다.
    EXPECT_NE(std::find(result->rescued_ids.begin(), result->rescued_ids.end(), 1),
              result->rescued_ids.end());

    // 결과 경로는 최종적으로 충돌이 없어야 한다(핵심 불변식).
    const Path& r_path = result->paths.at(0);
    const Path& k_path = result->paths.at(1);
    for (const auto& a : r_path) {
        for (const auto& b : k_path) EXPECT_FALSE(a == b);
    }
    EXPECT_EQ(r_path.back().x, 3);
    EXPECT_EQ(r_path.back().y, 0);
    EXPECT_EQ(k_path.back().x, 1);
    EXPECT_EQ(k_path.back().y, 2);
}

TEST(PBSReplanTest, FallsBackToFullReplanWhenEscalationCannotResolveConflict) {
    // 3x1 통로(폭 1, 우회 불가능): (0,0)-(1,0)-(2,0).
    // 로봇0(1순위): (2,0)->(2,0) 제자리, Tail로 (2,0)을 영원히 점유.
    // 로봇1(2순위): (0,0)->(1,0) — 처음엔 K와 안 겹치는 짧은 경로.
    // current_time=0에 (1,0)에 새 장애물이 생기면 로봇1만 영향받는데,
    // 폭 1 통로라 비켜갈 곳이 없어 Tier 0/1 모두 실패하고 안전망으로
    // 떨어져야 한다. 안전망도 (1,0)이 막혀 있고 다른 길이 없으므로 결국
    // 전체 실패(nullopt)한다 — "안전망까지 가도 못 풀 수 있다"를 보여준다.
    Map map(3, 1);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{2, 0}, Cell{2, 0}},
        Agent{1, Cell{0, 0}, Cell{1, 0}},
    };
    auto initial = pbs.plan(agents);
    ASSERT_TRUE(initial.has_value());

    auto result = pbs.replan(agents, *initial, {Cell{1, 0}}, /*current_time=*/0);

    EXPECT_FALSE(result.has_value());
}

TEST(PBSReplanTest, FailsEvenAfterFallbackWhenBlockersGoalIsTheOnlyDetour) {
    // 2x3 격자: (0,*) 세로 통로 + (1,*) 옆 칸.
    // 로봇0: (1,1)->(1,1) 제자리 — 목적지 자체가 (1,1)이므로 Tail
    //   Reservation으로 (1,1)을 영원히 점유해야 한다(이 보장이 register_path의
    //   전제다). 장애물과는 무관해서 처음엔 working_ids에 안 들어간다.
    // 로봇1: (0,0)->(0,2) — t=1에 (0,1)을 지나가는데, 그 자리에 새 장애물이
    //   생기면 유일한 우회로가 (1,1)뿐이다. 그런데 로봇0의 목적지가 정확히
    //   (1,1)이라서, Tier 1로 로봇0을 끌어와 재탐색해도 로봇0은 결국 다시
    //   (1,1)에 도착해 영원히 머물러야 한다 — "비켜줄 곳이 없는" 구조적
    //   충돌이다(06장 6.5절의 한계와 같은 종류). 그래서 Tier 0/1은 물론
    //   안전망(전체 재계획)까지도 실패해야 한다.
    Map map(2, 3);
    PBS pbs(map);

    std::vector<Agent> agents = {
        Agent{0, Cell{1, 1}, Cell{1, 1}},
        Agent{1, Cell{0, 0}, Cell{0, 2}},
    };
    auto initial = pbs.plan(agents);
    ASSERT_TRUE(initial.has_value());
    ASSERT_EQ(initial->at(1).back(), (SpaceTimeCell{0, 2, 2}));

    auto result = pbs.replan(agents, *initial, {Cell{0, 1}}, /*current_time=*/0);

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
