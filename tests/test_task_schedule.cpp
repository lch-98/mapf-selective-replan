// ─────────────────────────────────────────────────────────────────
// tests/test_task_schedule.cpp
//
// 17장 결정 3·9 — 근무표(TaskWindow, TaskSchedule) 테스트.
// ─────────────────────────────────────────────────────────────────
#include <gtest/gtest.h>

#include <stdexcept>

#include "mapf/task_schedule.hpp"

using namespace mapf;

namespace {

// start_time부터 한 스텝에 한 칸씩 cells를 지나는 경로를 만든다.
Path make_path(int start_time, const std::vector<Cell>& cells) {
    Path path;
    for (size_t i = 0; i < cells.size(); ++i) {
        path.push_back(SpaceTimeCell{cells[i].x, cells[i].y, start_time + static_cast<int>(i)});
    }
    return path;
}

}  // namespace

TEST(TaskScheduleTest, StateAtFollowsTheWindow) {
    // t2 도착, 작업 3 → t2~t4 작업 중, t5부터 주차 중(17장 Q3).
    TaskWindow window{2, 5};
    EXPECT_EQ(task_state_at(window, 1), TaskState::kMoving);
    EXPECT_EQ(task_state_at(window, 2), TaskState::kWorking);
    EXPECT_EQ(task_state_at(window, 4), TaskState::kWorking);
    EXPECT_EQ(task_state_at(window, 5), TaskState::kParked);
    EXPECT_EQ(task_state_at(window, 9), TaskState::kParked);

    // 작업 시간 0이면 "작업 중"인 순간이 없다 — 도착하자마자 주차 중이다.
    TaskWindow no_task{3, 3};
    EXPECT_EQ(task_state_at(no_task, 2), TaskState::kMoving);
    EXPECT_EQ(task_state_at(no_task, 3), TaskState::kParked);
}

TEST(TaskScheduleTest, WindowFromPathReadsArrivalAndEnd) {
    // (0,0)→(2,0), t2 도착, 작업 3: 경로 끝에 작업 칸이 붙어 t5에서 끝난다 → [2, 5].
    Agent worker{0, Cell{0, 0}, Cell{2, 0}, /*service_time=*/3};
    Path worked = make_path(0, {{0, 0}, {1, 0}, {2, 0}, {2, 0}, {2, 0}, {2, 0}});
    EXPECT_EQ(window_from_path(worker, worked), (TaskWindow{2, 5}));

    // 작업 시간 0: 도착 시각 = 작업 끝 → [2, 2].
    Agent mover{1, Cell{0, 0}, Cell{2, 0}};
    Path moved = make_path(0, {{0, 0}, {1, 0}, {2, 0}});
    EXPECT_EQ(window_from_path(mover, moved), (TaskWindow{2, 2}));

    // 경로가 t4부터 시작해도 시각은 경로의 t값을 따른다: t5 도착, 작업 1 → [5, 6].
    Agent late{2, Cell{0, 0}, Cell{1, 0}, /*service_time=*/1};
    Path late_path = make_path(4, {{0, 0}, {1, 0}, {1, 0}});
    EXPECT_EQ(window_from_path(late, late_path), (TaskWindow{5, 6}));
}

TEST(TaskScheduleTest, WindowFromPathRejectsPathWithoutServiceCells) {
    Agent worker{0, Cell{0, 0}, Cell{2, 0}, /*service_time=*/3};

    // 작업 칸 없이 도착만 한 경로(작업 시간 0으로 만든 경로)를 작업 시간 3인 로봇에 쓰면 안 된다.
    EXPECT_THROW(window_from_path(worker, make_path(0, {{0, 0}, {1, 0}, {2, 0}})),
                 std::invalid_argument);

    // 경로가 목적지에서 끝나지 않으면 안 된다.
    EXPECT_THROW(
        window_from_path(worker, make_path(0, {{0, 0}, {1, 0}, {2, 0}, {2, 0}, {2, 0}, {1, 0}})),
        std::invalid_argument);

    // 음수 작업 시간, 빈 경로도 안 된다.
    Agent negative{1, Cell{0, 0}, Cell{0, 0}, /*service_time=*/-1};
    EXPECT_THROW(window_from_path(negative, make_path(0, {{0, 0}})), std::invalid_argument);
    EXPECT_THROW(window_from_path(worker, Path{}), std::invalid_argument);
}

TEST(TaskScheduleTest, RescuedPathCannotBeReadAsAFreshPlan) {
    // 17장 결정 3의 예시. K는 t0에 (4,1)에 도착, 작업 5 → 실제 작업 구간은 [0, 5].
    // t6에 구조 로봇으로 불려 t7에 (4,0)으로 비켰다가 t8에 돌아왔다.
    // 이 경로에서 "경로 끝 - 작업 시간"으로 읽으면 [3, 8]로 틀리게 나온다 — 그래서
    // window_from_path는 이 경로를 거절해야 한다(끝의 6칸 중 t7이 목적지가 아님).
    Agent k{0, Cell{4, 1}, Cell{4, 1}, /*service_time=*/5};
    Path rescued = make_path(0, {{4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1},
                                 {4, 0}, {4, 1}});
    EXPECT_THROW(window_from_path(k, rescued), std::invalid_argument);
}

TEST(TaskScheduleTest, ScheduleFromPlanCoversEveryAgent) {
    std::vector<Agent> agents = {
        Agent{0, Cell{0, 0}, Cell{2, 0}, /*service_time=*/3},
        Agent{1, Cell{4, 1}, Cell{4, 1}},
    };
    std::unordered_map<int, Path> paths;
    paths[0] = make_path(0, {{0, 0}, {1, 0}, {2, 0}, {2, 0}, {2, 0}, {2, 0}});
    paths[1] = make_path(0, {{4, 1}});

    TaskSchedule schedule = schedule_from_plan(agents, paths);

    ASSERT_EQ(schedule.size(), 2u);
    EXPECT_EQ(schedule.at(0), (TaskWindow{2, 5}));
    EXPECT_EQ(schedule.at(1), (TaskWindow{0, 0}));
}

TEST(TaskScheduleTest, UpdateScheduleRecomputesOnlyAgentsStillMoving) {
    // 재계획 시각 T = 6.
    //   K: [0, 5] → T=6에 주차 중. 구조되어 비켰다가 돌아온 경로를 받아도 근무표는 그대로여야 한다.
    //   M: [4, 9] → T=6에 작업 중. 경로가 어떻게 바뀌어도 그대로.
    //   W: [8, 10] → T=6에 아직 이동 중. 새 경로(t12 도착, 작업 2 → t14 끝)로 다시 계산 → [12, 14].
    std::vector<Agent> agents = {
        Agent{0, Cell{4, 1}, Cell{4, 1}, /*service_time=*/5},  // K
        Agent{1, Cell{0, 0}, Cell{3, 0}, /*service_time=*/5},  // M
        Agent{2, Cell{0, 2}, Cell{6, 2}, /*service_time=*/2},  // W
    };
    TaskSchedule previous;
    previous[0] = TaskWindow{0, 5};
    previous[1] = TaskWindow{4, 9};
    previous[2] = TaskWindow{8, 10};

    std::unordered_map<int, Path> new_paths;
    new_paths[0] = make_path(0, {{4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1},
                                 {4, 0}, {4, 1}});
    new_paths[1] = make_path(0, {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {3, 0}, {3, 0}, {3, 0},
                                 {3, 0}, {3, 0}, {3, 0}});
    new_paths[2] = make_path(0, {{0, 2}, {1, 2}, {2, 2}, {3, 2}, {3, 1}, {3, 1}, {3, 1},
                                 {3, 1}, {3, 1}, {4, 1}, {5, 1}, {5, 2}, {6, 2}, {6, 2},
                                 {6, 2}});

    TaskSchedule updated = update_schedule(agents, previous, new_paths, /*current_time=*/6);

    EXPECT_EQ(updated.at(0), (TaskWindow{0, 5}));    // 구조된 주차 로봇: 그대로
    EXPECT_EQ(updated.at(1), (TaskWindow{4, 9}));    // 작업 중: 그대로
    EXPECT_EQ(updated.at(2), (TaskWindow{12, 14}));  // 이동 중이던 로봇: 새 도착으로
}

TEST(TaskScheduleTest, UpdateScheduleKeepsRescuedAgentThatDidNotMove) {
    // window_from_path의 한계와, 그것을 update_schedule이 막는다는 것을 확인한다.
    // K는 t0에 도착, 작업 5 → 실제 작업 구간 [0, 5]. t6에 구조 로봇으로 불렸지만 비킬 필요가
    // 없어서 그대로 머물렀다(헛부탁). 새 경로는 t0~t6 전부 (4,1)이다.
    Agent k{0, Cell{4, 1}, Cell{4, 1}, /*service_time=*/5};
    Path stayed = make_path(0, {{4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1}, {4, 1}});

    // window_from_path로 읽으면 끝의 6칸이 전부 목적지라 검사를 통과하고 [1, 6]으로 틀리게 읽는다.
    EXPECT_EQ(window_from_path(k, stayed), (TaskWindow{1, 6}));

    // update_schedule은 이미 도착한(주차 중인) 로봇의 경로를 다시 읽지 않으므로 [0, 5]를 지킨다.
    TaskSchedule previous;
    previous[0] = TaskWindow{0, 5};
    std::unordered_map<int, Path> new_paths;
    new_paths[0] = stayed;
    TaskSchedule updated = update_schedule({k}, previous, new_paths, /*current_time=*/6);
    EXPECT_EQ(updated.at(0), (TaskWindow{0, 5}));
}
