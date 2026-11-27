// ADG: 손으로 만든 시간표로 동작 목록과 화살표, 내줄 수 있는 동작을 확인한다.
#include <gtest/gtest.h>

#include <algorithm>

#include "mapf_exec/adg.hpp"

using namespace mapf_exec;
using Kind = AdgAction::Kind;

namespace {
// 교차로 (31.6 예 2와 같은 모양, 칸은 [열, 행])
//   C: 1행을 왼→오. 번호 0에 1행 0열 → 1행 1열(X), 번호 1에 X → 1행 2열
//   D: 0행 1열에서 번호 2까지 대기, 번호 2에 X로, 번호 3에 2행 1열로
const std::map<int, std::vector<GridCell>> kCross = {
    {0, {{0, 1}, {1, 1}, {2, 1}}},
    {1, {{1, 0}, {1, 0}, {1, 0}, {1, 1}, {1, 2}}},
};
}  // namespace

TEST(Adg, ActionsDropWaitsAndKeepPlanTime) {
    Adg g(kCross, {});
    const auto& d = g.robot_actions(1);
    ASSERT_EQ(d.size(), 2u);  // 대기 2번은 동작이 아님
    EXPECT_EQ(g.actions()[d[0]].from, (GridCell{1, 0}));
    EXPECT_EQ(g.actions()[d[0]].to, (GridCell{1, 1}));
    EXPECT_EQ(g.actions()[d[0]].plan_t, 2);
    EXPECT_EQ(g.actions()[d[1]].plan_t, 3);
}

TEST(Adg, CrossingGetsOneEdgeFromLeavingToEntering) {
    Adg g(kCross, {});
    EXPECT_EQ(g.num_cross_edges(), 1);
    const int c1 = g.robot_actions(0)[1];  // C: X → 1행 2열 (번호 1)
    const int d0 = g.robot_actions(1)[0];  // D: 0행 1열 → X (번호 2)
    ASSERT_EQ(g.cross_preds(d0).size(), 1u);
    EXPECT_EQ(g.cross_preds(d0)[0], c1);
}

TEST(Adg, ReleaseOrderFollowsPromises) {
    Adg g(kCross, {});
    const int c0 = g.robot_actions(0)[0], c1 = g.robot_actions(0)[1], d0 = g.robot_actions(1)[0];
    EXPECT_EQ(g.releasable(), std::vector<int>{c0});  // D는 C가 X를 떠날 때까지 못 감
    g.release(c0);
    EXPECT_TRUE(g.releasable().empty());  // C가 하는 중이면 C의 다음 동작도 안 나감
    g.done(c0);
    EXPECT_EQ(g.releasable(), std::vector<int>{c1});
    g.release(c1);
    EXPECT_TRUE(g.releasable().empty());  // C가 X를 다 떠나기 전(하는 중)에는 D 못 감
    g.done(c1);
    EXPECT_EQ(g.releasable(), std::vector<int>{d0});
    g.release(d0);
    g.done(d0);
    const int d1 = g.robot_actions(1)[1];
    EXPECT_EQ(g.releasable(), std::vector<int>{d1});
    g.release(d1);
    g.done(d1);
    EXPECT_TRUE(g.finished());
}

TEST(Adg, FollowingInSameStepNeedsLeaderToFinish) {
    // 같은 번호 0에: 로봇 0이 0행 1열 → 0행 2열로 나가고, 로봇 1이 0행 0열 → 0행 1열로 들어감 (계획상 허용되는 뒤따르기)
    Adg g({{0, {{1, 0}, {2, 0}}}, {1, {{0, 0}, {1, 0}}}}, {});
    EXPECT_EQ(g.num_cross_edges(), 1);
    const int lead = g.robot_actions(0)[0], follow = g.robot_actions(1)[0];
    EXPECT_EQ(g.releasable(), std::vector<int>{lead});
    g.release(lead);
    g.done(lead);
    EXPECT_EQ(g.releasable(), std::vector<int>{follow});
}

TEST(Adg, NoEdgeWhenOtherUsedCellLongBefore) {
    // 로봇 0이 번호 0에 0행 0열을 떠남. 로봇 1은 번호 5에 0행 0열로 들어옴 → 화살표 있음(먼저 떠나고 나중에 들어옴)
    // 로봇 1이 번호 0에 0행 3열을 떠나고, 로봇 0은 그 칸에 안 감 → 화살표 없음
    Adg g({{0, {{0, 0}, {0, 1}}}, {1, {{3, 0}, {2, 0}, {1, 0}, {1, 0}, {1, 0}, {1, 0}, {0, 0}}}}, {});
    EXPECT_EQ(g.num_cross_edges(), 1);
    const int r1_last = g.robot_actions(1).back();
    EXPECT_EQ(g.cross_preds(r1_last), std::vector<int>{g.robot_actions(0)[0]});
}

TEST(Adg, ServiceActionAtGoal) {
    // 1행 0열 → 1행 1열 도착(번호 1) 후 작업 2스텝(번호 1~3에 목적지)
    Adg g({{0, {{0, 1}, {1, 1}, {1, 1}, {1, 1}}}}, {{0, 2}});
    const auto& r = g.robot_actions(0);
    ASSERT_EQ(r.size(), 2u);
    const auto& s = g.actions()[r[1]];
    EXPECT_EQ(s.kind, Kind::kService);
    EXPECT_EQ(s.service_steps, 2);
    EXPECT_EQ(s.plan_t, 1);
    EXPECT_EQ(s.from, (GridCell{1, 1}));
}

TEST(Adg, StandingRobotHasNoActions) {
    Adg g({{0, {{2, 2}}}, {1, {{0, 0}, {1, 0}}}}, {});
    EXPECT_TRUE(g.robot_actions(0).empty());
    EXPECT_EQ(g.actions().size(), 1u);
}

TEST(Adg, MutationSwitchRemovesCrossEdges) {
    Adg g(kCross, {}, /*cross_edges=*/false);
    EXPECT_EQ(g.num_cross_edges(), 0);
    auto r = g.releasable();
    std::sort(r.begin(), r.end());
    EXPECT_EQ(r.size(), 2u);  // 약속이 없으면 C와 D가 동시에 출발(실험에 쓰면 안 되는 이유)
}

TEST(Adg, MisuseThrows) {
    Adg g(kCross, {});
    const int c0 = g.robot_actions(0)[0];
    EXPECT_THROW(g.done(c0), std::logic_error);
    g.release(c0);
    EXPECT_THROW(g.release(c0), std::logic_error);
}

TEST(Adg, ServiceInTheMiddleWhenStartGiven) {
    // 재계획 뒤 "작업 2스텝 → 비켜 주는 이동": 4행 7열에서 번호 0~2 작업, 번호 2에 4행 8열로
    Adg g({{0, {{7, 4}, {7, 4}, {7, 4}, {8, 4}}}}, {{0, 2}}, true, {{0, 0}});
    const auto& r = g.robot_actions(0);
    ASSERT_EQ(r.size(), 2u);
    EXPECT_EQ(g.actions()[r[0]].kind, Kind::kService);
    EXPECT_EQ(g.actions()[r[0]].index, 0);
    EXPECT_EQ(g.actions()[r[1]].kind, Kind::kMove);
    EXPECT_EQ(g.actions()[r[1]].index, 1);
    EXPECT_EQ(g.actions()[r[1]].to, (GridCell{8, 4}));
}

// ── 곧게 지나가기(31.24): 하는 중인 이동의 다음 동작이 같은 방향이고 약속이 다 끝났으면 미리 내줄 수 있다 ──
TEST(Adg, StraightNextOnlyForSameDirectionWhileMoving) {
    Adg g(kCross, {});
    const int c0 = g.robot_actions(0)[0], c1 = g.robot_actions(0)[1];
    EXPECT_EQ(g.straight_next(0), -1);  // 하는 중인 동작이 없음
    g.release(c0);
    EXPECT_EQ(g.straight_next(0), c1);  // 1행 0열 → 1열 → 2열: 같은 방향
    g.release(c1);                       // 미리 내줌
    EXPECT_EQ(g.straight_next(0), -1);  // 한 칸 앞까지만
    EXPECT_TRUE(std::find(g.releasable().begin(), g.releasable().end(), c1) == g.releasable().end());
    g.done(c0);
    EXPECT_TRUE(std::find(g.releasable().begin(), g.releasable().end(), c1) == g.releasable().end());  // 두 번 안 내줌
    g.done(c1);
    // 꺾는 길: 0행 0열 → 0행 1열 → 1행 1열
    Adg t({{0, {{0, 0}, {1, 0}, {1, 1}}}}, {});
    t.release(t.robot_actions(0)[0]);
    EXPECT_EQ(t.straight_next(0), -1);
    // 다음이 작업 동작이면 아님
    Adg s({{0, {{0, 0}, {1, 0}}}}, {{0, 2}});
    s.release(s.robot_actions(0)[0]);
    EXPECT_EQ(s.straight_next(0), -1);
}

TEST(Adg, StraightNextWaitsForPromises) {
    // A: 0행을 0열 → 2열. B는 0행 2열에 있다가 번호 0에 1행 2열로 → A가 0행 2열로 들어가는 동작(번호 1)은 B가 떠난 뒤
    Adg g({{0, {{0, 0}, {1, 0}, {2, 0}}}, {1, {{2, 0}, {2, 1}, {2, 1}}}}, {});
    const int a0 = g.robot_actions(0)[0], a1 = g.robot_actions(0)[1], b0 = g.robot_actions(1)[0];
    ASSERT_EQ(g.cross_preds(a1), std::vector<int>{b0});
    g.release(a0);
    EXPECT_EQ(g.straight_next(0), -1);  // B가 아직 안 떠남
    g.release(b0);
    EXPECT_EQ(g.straight_next(0), -1);  // 떠나는 중(아직 다음 칸 중심 도착 전)
    g.done(b0);
    EXPECT_EQ(g.straight_next(0), a1);
}

TEST(Adg, UnreleaseReturnsActionToNotYet) {
    Adg g(kCross, {});
    const int c0 = g.robot_actions(0)[0], c1 = g.robot_actions(0)[1];
    g.release(c0);
    g.release(c1);
    g.unrelease(c1);  // 미리 내준 것을 취소(예: 그 칸에서 장애물 발견)
    EXPECT_FALSE(g.released(c1));
    g.done(c0);
    EXPECT_EQ(g.releasable(), std::vector<int>{c1});
    EXPECT_THROW(g.unrelease(c0), std::logic_error);  // 끝난 동작은 취소 못 함
}
