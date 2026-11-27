// 번호 다시 붙이기: docs/31 31.17의 A·C·D·E 예시 표를 그대로 확인한다. 칸은 {열 x, 행 y}.
#include <gtest/gtest.h>

#include "mapf_exec/plan_check.hpp"
#include "mapf_exec/retime.hpp"

using namespace mapf_exec;

namespace {

// 옛 계획(31.17 그림과 같은 모양을 만들기 위한 시간표). 로봇 번호: A=0, C=1, D=2, E=3
//   A: 1행을 동쪽으로 1열 → 7열
//   C: 3행을 동쪽으로 1열 → 6열, 번호 3에 X(3행 4열)를 떠남
//   D: 2행 4열에서 기다렸다가 번호 5에 X로, 번호 6에 4행 4열로 (약속: C가 X를 떠난 뒤)
//   E: 5행 7열 → 4행 7열 도착 후 작업 2스텝
const std::map<int, std::vector<GridCell>> kOld = {
    {0, {{1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 1}}},
    {1, {{1, 3}, {2, 3}, {3, 3}, {4, 3}, {5, 3}, {6, 3}}},
    {2, {{4, 2}, {4, 2}, {4, 2}, {4, 2}, {4, 2}, {4, 2}, {4, 3}, {4, 4}}},
    {3, {{7, 5}, {7, 4}, {7, 4}, {7, 4}}},
};

// 지금 모습(31.17 ②): A는 3열에 멈춤(동작 2개 끝), C는 3열 도착(2개 끝), D는 대기 중(0개), E는 작업 중(남은 2스텝)
Adg snapshot_adg(bool cross = true) {
    Adg g(kOld, {{3, 2}}, cross);
    for (int k = 0; k < 2; ++k) {
        g.release(g.robot_actions(0)[k]);
        g.done(g.robot_actions(0)[k]);
        g.release(g.robot_actions(1)[k]);
        g.done(g.robot_actions(1)[k]);
    }
    g.release(g.robot_actions(3)[0]);
    g.done(g.robot_actions(3)[0]);
    g.release(g.robot_actions(3)[1]);  // 작업 동작 하는 중
    return g;
}

const std::map<int, GridCell> kNow = {{0, {3, 1}}, {1, {3, 3}}, {2, {4, 2}}, {3, {7, 4}}};

}  // namespace

TEST(Retime, MatchesTableIn31_17) {
    auto r = retime(snapshot_adg(), kNow, {{3, 2}});
    // A: 약속 없음 → 한 칸씩
    EXPECT_EQ(r.paths[0], (std::vector<GridCell>{{3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 1}}));
    // C: 한 칸씩, 6열에서 끝
    std::vector<GridCell> c = r.paths[1];
    c.resize(4);
    EXPECT_EQ(c, (std::vector<GridCell>{{3, 3}, {4, 3}, {5, 3}, {6, 3}}));
    // D: C가 번호 2에 X를 떠남 → D는 번호 3에 X, 번호 4에 4행 4열
    EXPECT_EQ(r.paths[2], (std::vector<GridCell>{{4, 2}, {4, 2}, {4, 2}, {4, 3}, {4, 4}}));
    // E: 남은 작업 2스텝(번호 0~2 제자리)
    EXPECT_EQ(r.service_start.at(3), 0);
    EXPECT_EQ(r.service_steps.at(3), 2);
    EXPECT_EQ(r.paths[3][2], (GridCell{7, 4}));
}

TEST(Retime, ResultPassesPlanCheck) {
    const std::vector<std::string> rows(6, std::string(9, '.'));
    auto r = retime(snapshot_adg(), kNow, {{3, 2}});
    auto v = check_plan(rows, r.paths);
    EXPECT_TRUE(v.empty()) << v.front().describe();
}

TEST(Retime, WithoutPromisesTheNaiveTimetableCollides) {
    // 31.17 "그냥 붙이면": 약속 없이 붙이면 번호 1에 C와 D가 둘 다 X → 계획 검사가 잡음
    const std::vector<std::string> rows(6, std::string(9, '.'));
    auto r = retime(snapshot_adg(false), kNow, {{3, 2}});
    auto v = check_plan(rows, r.paths);
    ASSERT_FALSE(v.empty());
    EXPECT_EQ(v.front().kind, ViolationKind::kVertex);
}

TEST(Retime, NobodyLateGivesSameCellOrderAndTimesWithoutFollowing) {
    // 아무것도 안 했을 때(번호 0 = 옛 계획 번호 0) 다시 붙이면: 뒤따르기가 없는 이 예에서는 칸 순서가 같고,
    // A·C는 옛 계획과 번호까지 같다(D는 옛 계획의 대기 대신 약속이 정한 만큼만 기다림)
    Adg g(kOld, {{3, 2}});
    auto r = retime(g, {{0, {1, 1}}, {1, {1, 3}}, {2, {4, 2}}, {3, {7, 5}}}, {});
    EXPECT_EQ(r.paths[0], kOld.at(0));
    std::vector<GridCell> c = r.paths[1];
    c.resize(kOld.at(1).size());
    EXPECT_EQ(c, kOld.at(1));
    EXPECT_TRUE(check_plan(std::vector<std::string>(6, std::string(9, '.')), r.paths).empty());
    EXPECT_EQ(r.service_steps.at(3), 2);
    EXPECT_EQ(r.service_start.at(3), 1);  // 번호 1에 도착 후 작업 시작
}

TEST(Retime, MovingRobotIsRejected) {
    Adg g(kOld, {});
    g.release(g.robot_actions(0)[0]);  // 이동 중
    EXPECT_THROW(retime(g, {{0, {1, 1}}, {1, {1, 3}}, {2, {4, 2}}, {3, {7, 5}}}, {}), std::logic_error);
}

TEST(Retime, WorkingRobotWaitsEvenIfWorkIsFromEarlierPlan) {
    // 앞선 재계획 때 이미 작업 중이던 로봇: 새 ADG에는 "작업 뒤 비켜 주는 이동"(4행 7열 → 4행 8열)만 있다.
    // 실행기가 남은 작업 2스텝을 알려 주면, 그 이동은 번호 2 뒤에야 한다(작업 중인 로봇은 절대 안 움직임).
    Adg g({{3, {{7, 4}, {8, 4}}}}, {});
    auto r = retime(g, {{3, {7, 4}}}, {{3, 2}});
    EXPECT_EQ(r.paths[3], (std::vector<GridCell>{{7, 4}, {7, 4}, {7, 4}, {8, 4}}));
    EXPECT_EQ(r.service_start.at(3), 0);
    EXPECT_EQ(r.service_steps.at(3), 2);
}
