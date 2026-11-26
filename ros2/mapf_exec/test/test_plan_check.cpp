// 계획 검사: 걸려야 하는 예와 통과해야 하는 예를 손으로 만든다.
// 격자 3행 3열, 1행 1열만 선반:
//   0행  . . .
//   1행  . # .
//   2행  . . .
#include <gtest/gtest.h>

#include "mapf_exec/plan_check.hpp"

using namespace mapf_exec;

namespace {
const std::vector<std::string> kRows = {"...", ".#.", "..."};

bool has(const std::vector<Violation>& v, ViolationKind k) {
    for (const auto& x : v) {
        if (x.kind == k) return true;
    }
    return false;
}
}  // namespace

TEST(PlanCheck, ValidPlanPasses) {
    // 로봇 0: 0행을 왼→오, 로봇 1: 2행을 왼→오
    auto v = check_plan(kRows, {{0, {{0, 0}, {1, 0}, {2, 0}}}, {1, {{0, 2}, {1, 2}, {2, 2}}}});
    EXPECT_TRUE(v.empty()) << v.front().describe();
}

TEST(PlanCheck, FollowingIntoJustLeftCellIsAllowed) {
    // 로봇 0이 0행 1열 → 0행 2열로 나가는 같은 번호에 로봇 1이 0행 0열 → 0행 1열로 들어감 (코어 규칙상 허용)
    auto v = check_plan(kRows, {{0, {{1, 0}, {2, 0}}}, {1, {{0, 0}, {1, 0}}}});
    EXPECT_TRUE(v.empty()) << v.front().describe();
}

TEST(PlanCheck, BlockedOffMapJump) {
    EXPECT_TRUE(has(check_plan(kRows, {{0, {{1, 0}, {1, 1}}}}), ViolationKind::kBlocked));
    EXPECT_TRUE(has(check_plan(kRows, {{0, {{2, 0}, {3, 0}}}}), ViolationKind::kOffMap));
    EXPECT_TRUE(has(check_plan(kRows, {{0, {{0, 0}, {2, 0}}}}), ViolationKind::kJump));
    EXPECT_TRUE(has(check_plan(kRows, {{0, {{0, 0}, {1, 1}}}}), ViolationKind::kJump));  // 대각선
    EXPECT_TRUE(has(check_plan(kRows, {{0, {}}}), ViolationKind::kEmptyPath));
}

TEST(PlanCheck, VertexConflict) {
    // 번호 1에 둘 다 0행 1열
    auto v = check_plan(kRows, {{0, {{0, 0}, {1, 0}}}, {1, {{2, 0}, {1, 0}}}});
    ASSERT_TRUE(has(v, ViolationKind::kVertex));
    EXPECT_EQ(v.front().t, 1);
}

TEST(PlanCheck, VertexConflictWithRobotStayingAtGoal) {
    // 로봇 0은 0행 0열에서 끝나 그대로 머무름. 로봇 1이 번호 1에 그 칸으로 들어감
    auto v = check_plan(kRows, {{0, {{0, 0}}}, {1, {{1, 0}, {0, 0}}}});
    EXPECT_TRUE(has(v, ViolationKind::kVertex));
}

TEST(PlanCheck, SwapConflict) {
    auto v = check_plan(kRows, {{0, {{0, 0}, {1, 0}}}, {1, {{1, 0}, {0, 0}}}});
    EXPECT_TRUE(has(v, ViolationKind::kSwap));
    EXPECT_FALSE(has(v, ViolationKind::kVertex));
}

TEST(PlanCheck, ObstacleFromItsTime) {
    const std::map<int, std::vector<GridCell>> p = {{0, {{0, 0}, {1, 0}, {2, 0}}}};
    EXPECT_TRUE(has(check_plan(kRows, p, {{{2, 0}, 2}}), ViolationKind::kObstacle));   // 번호 2에 밟음
    EXPECT_TRUE(has(check_plan(kRows, p, {{{2, 0}, 5}}), ViolationKind::kObstacle));   // 도착 후 머무는 칸
    EXPECT_FALSE(has(check_plan(kRows, p, {{{0, 0}, 1}}), ViolationKind::kObstacle));  // 이미 떠난 칸
}
