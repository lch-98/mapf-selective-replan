// ─────────────────────────────────────────────────────────────────
// tests/test_space_time_astar.cpp
//
// 04_space_time_astar.md에서 설명한 SpaceTimeAStar의 동작을 확인한다.
// ─────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>

#include <algorithm>
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

// ───────────────────────────────────────────────────────────────
// 18장 2-2단계 — 장부가 굳은 뒤에는 같은 칸을 한 번만 확인
// ───────────────────────────────────────────────────────────────

namespace {

// 막힌 칸 목록에서 주인(로봇 id)을 처음 나온 순서대로 뽑는다(장애물 -1은 뺀다). 구조 로봇을 고를 때
// 쓰는 것과 같은 정보다.
std::vector<int> owners_in_order(const AStarResult& result, const ReservationTable& table) {
    std::vector<int> owners;
    for (const SpaceTimeCell& c : result.blocked_attempts) {
        std::optional<int> owner = table.get_owner(c.x, c.y, c.t);
        if (!owner.has_value() || *owner == -1) continue;
        if (std::find(owners.begin(), owners.end(), *owner) == owners.end()) owners.push_back(*owner);
    }
    return owners;
}

}  // namespace

TEST(SpaceTimeAStarTest, SettledTableChecksEachCellOnceAfterItSettles) {
    // ExpandedCountsEveryStateCheckedBeforeFailing과 같은 상황: 1x3 통로, 가운데 (1,0)이 t0부터 끝까지
    // 막혀 있다(다른 로봇이 서 있음). 장부는 t0부터 한 번도 안 바뀌므로 굳는 시각은 0이다.
    //   - 굳는 시각을 모르면: 출발 칸 (0,0)을 t0~t10까지 11번 확인한 뒤 실패.
    //   - 굳는 시각 0을 알려 주면: (0,0)에서 기다려 봐야 같은 그림이므로 1번만 확인하고 실패.
    Map map(3, 1);
    AStarConfig config;
    config.max_timestep = 10;
    ReservationTable unknown;
    for (int t = 0; t <= config.max_timestep; ++t) unknown.reserve(1, 0, t, /*agent_id=*/7);
    ReservationTable settled = unknown;
    settled.mark_settled_from(0);

    AStarResult slow = SpaceTimeAStar(map, unknown, config).search_with_diagnostics(Cell{0, 0}, Cell{2, 0});
    AStarResult fast = SpaceTimeAStar(map, settled, config).search_with_diagnostics(Cell{0, 0}, Cell{2, 0});

    EXPECT_FALSE(slow.path.has_value());
    EXPECT_FALSE(fast.path.has_value());
    EXPECT_EQ(slow.expanded, 11);
    EXPECT_EQ(fast.expanded, 1);
    EXPECT_EQ(owners_in_order(fast, settled), owners_in_order(slow, unknown));  // 둘 다 {7}
}

TEST(SpaceTimeAStarTest, SettledTableKeepsPathWhenWaitingIsNeededBeforeItSettles) {
    // 5x1 통로, (0,0) → (4,0). 다른 로봇(7번)이 t0~t3 동안 (2,0)에 있다가 사라진다(장부가 t4에 바뀜 →
    // 굳는 시각 4). 로봇은 굳기 전에 기다려야 한다: (2,0)에 가장 빨리 t4에 들어가 t6에 도착.
    // 굳기 전에는 지금처럼 시각마다 확인하므로, 굳는 시각을 알려 줘도 경로가 똑같아야 한다.
    Map map(5, 1);
    ReservationTable unknown;
    for (int t = 0; t <= 3; ++t) unknown.reserve(2, 0, t, /*agent_id=*/7);
    ReservationTable settled = unknown;
    settled.mark_settled_from(4);

    AStarResult a = SpaceTimeAStar(map, unknown).search_with_diagnostics(Cell{0, 0}, Cell{4, 0});
    AStarResult b = SpaceTimeAStar(map, settled).search_with_diagnostics(Cell{0, 0}, Cell{4, 0});

    ASSERT_TRUE(a.path.has_value());
    ASSERT_TRUE(b.path.has_value());
    EXPECT_EQ(*a.path, *b.path);
    EXPECT_EQ(b.path->back(), (SpaceTimeCell{4, 0, 6}));
    EXPECT_LE(b.expanded, a.expanded);
}

TEST(SpaceTimeAStarTest, SettledTableBoundsWorkWhenEnclosedAndKeepsBlockers) {
    //   y=0:  . . . . .     ← 로봇이 갇히는 윗줄(빈 칸 5개). (4,0)은 8번 로봇이 t2~t4에 지나감
    //   y=1:  # # . # #     ← 유일한 통로 (2,1)에 7번 로봇이 t0부터 끝까지 서 있음
    //   y=2:  . . . . .
    // 로봇은 (0,0) → (0,2). 7번 때문에 절대 못 간다. 장부는 8번이 사라지는 t5부터 굳는다.
    //   - 굳는 시각을 모르면: 윗줄 5칸을 t0~t20의 모든 시각마다 확인한다.
    //   - 굳는 시각 5를 알려 주면: t0~t5의 6개 층만 확인 → 5칸 × 6층 = 30개 이하.
    // 그리고 막은 로봇 목록(구조 후보)과 그 순서는 같아야 한다(18장 18.4).
    Map map(std::vector<std::string>{
        ".....",
        "##.##",
        ".....",
    });
    AStarConfig config;
    config.max_timestep = 20;
    ReservationTable unknown;
    for (int t = 0; t <= config.max_timestep; ++t) unknown.reserve(2, 1, t, /*agent_id=*/7);
    for (int t = 2; t <= 4; ++t) unknown.reserve(4, 0, t, /*agent_id=*/8);
    ReservationTable settled = unknown;
    settled.mark_settled_from(5);

    AStarResult slow = SpaceTimeAStar(map, unknown, config).search_with_diagnostics(Cell{0, 0}, Cell{0, 2});
    AStarResult fast = SpaceTimeAStar(map, settled, config).search_with_diagnostics(Cell{0, 0}, Cell{0, 2});

    EXPECT_FALSE(slow.path.has_value());
    EXPECT_FALSE(fast.path.has_value());
    EXPECT_LE(fast.expanded, 5 * 6);
    EXPECT_LT(fast.expanded, slow.expanded);
    EXPECT_EQ(owners_in_order(fast, settled), owners_in_order(slow, unknown));
}

// ───────────────────────────────────────────────────────────────
// 21장 5-5 — 동점 규칙 스위치(AStarConfig::tie_break). 어느 규칙이든 경로 길이는 같고,
// "길이가 같은 경로 중 어느 것을, 얼마나 많이 확인해서" 찾는지만 달라진다.
// ───────────────────────────────────────────────────────────────

TEST(SpaceTimeAStarTest, SmallerGTieBreakKeepsLengthButChecksEveryTiedState) {
    // TieBreakGoesStraightAndIsFixed와 같은 빈 3x3, (0,0) → (2,2). 최단 경로 위의 칸은 전부 f = 4다.
    //   - g 큰 쪽 먼저(기본): 곧장 파고들어 5개만 확인한다.
    //   - g 작은 쪽 먼저: f = 4인 칸을 출발에서 가까운 층부터 다 확인한 뒤에야 목적지(g = 4)를 꺼낸다
    //     → 9칸 전부. 기다리기 상태는 f = 5 이상이라 확인하지 않는다.
    // 길이(도착 t4)는 같아야 한다.
    Map map(3, 3);
    ReservationTable reservations;
    AStarConfig config;
    config.tie_break = TieBreak::kSmallerG;
    SpaceTimeAStar astar(map, reservations, config);

    AStarResult result = astar.search_with_diagnostics(Cell{0, 0}, Cell{2, 2});

    ASSERT_TRUE(result.path.has_value());
    EXPECT_EQ(result.path->back(), (SpaceTimeCell{2, 2, 4}));
    EXPECT_EQ(result.expanded, 9);
}

TEST(SpaceTimeAStarTest, RandomTieBreakKeepsLengthAndIsReproducible) {
    // 빈 5x5, (0,0) → (4,4): 길이 8인 최단 경로가 70개다. 무작위 동점 규칙은
    //   - 씨앗이 무엇이든 길이 8(도착 t8)을 찾고,
    //   - 같은 씨앗이면 몇 번을 돌려도 같은 경로,
    //   - 씨앗이 다르면 다른 경로가 나올 수 있어야 한다(20개 씨앗에서 2가지 이상).
    Map map(5, 5);
    ReservationTable reservations;
    std::vector<Path> found;
    for (std::uint32_t seed = 0; seed < 20; ++seed) {
        AStarConfig config;
        config.tie_break = TieBreak::kRandom;
        config.tie_seed = seed;
        SpaceTimeAStar astar(map, reservations, config);
        std::optional<Path> first = astar.search(Cell{0, 0}, Cell{4, 4});
        ASSERT_TRUE(first.has_value());
        EXPECT_EQ(first->back(), (SpaceTimeCell{4, 4, 8}));
        EXPECT_EQ(astar.search(Cell{0, 0}, Cell{4, 4}), first);
        if (std::find(found.begin(), found.end(), *first) == found.end()) found.push_back(*first);
    }
    EXPECT_GE(found.size(), 2u);
}
